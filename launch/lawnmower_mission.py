#!/usr/bin/env python3
"""SkyScan mission: read the start QR, sweep the room, stop on the matching QR.

Talks to PX4 SITL over MAVLink (pymavlink, udp 14540) in offboard mode.
1. Take off and hover over the start QR (the reference), then ask qr_match_node
   to learn it from the camera (/qr_match_node/capture_reference).
2. Fly the corridor and lawnmower-sweep the room. /qr_match_node/matched going
   true over a candidate stops the sweep; the drone reports where and lands.
Matches are ignored near the pad, since the start QR matches itself.

World frame (Gazebo, X east / Y north) -> PX4 local NED, origin at the pad
(world 0,20):  north = Y - 20,  east = X,  down = -altitude.

Run (after the sim and skyscan_qr.launch.py are up):
    source /opt/ros/jazzy/setup.bash
    source QR_Node/ros2_ws/install/setup.bash
    python3 launch/lawnmower_mission.py            # add --force-arm if preflight blocks arming
"""

import argparse
import atexit
import math
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from pymavlink import mavutil
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from qr_match_interfaces.msg import QRResult

PAD_Y = 20.0
ROOM_X = (14.0, 52.0)          # sweep limits, 1 m inside the walls
ROOM_Y = (2.5, 37.5)
ROW_SPACING = 5.0              # camera footprint at 3 m is ~7 x 5.4 m
ZONE = (33.0, 14.0)            # red zone centre, 8 x 8 m
RED = dict(x=(ZONE[0] - 5.5, ZONE[0] + 5.5),   # keep-out box = zone + 1.5 m margin
           y=(ZONE[1] - 5.5, ZONE[1] + 5.5))
KNOWN_QR = {"qr_a": (20, 32), "qr_b": (18, 8), "qr_c": (24, 4),
            "qr_d": (46, 7.5)}   # sweep order c, d, b, a; qr_b is the reference
START_QR = (2.0, PAD_Y)        # reference QR, 2 m ahead of the pad
MATCH_MIN_X = 13.5             # only trust matches once inside the room


def world_to_ned(x, y, alt):
    return (y - PAD_Y, x, -alt)


def in_band(y):
    return RED["y"][0] <= y <= RED["y"][1]


def build_route():
    """Corridor transit, then a serpentine over the room that skirts the red zone."""
    route = [(2.0, 21.75), (13.0, 21.75), (ROOM_X[0], 21.75)]  # forward lane, clear
    ys, y = [], ROOM_Y[0]
    while y <= ROOM_Y[1] + 1e-6:
        ys.append(y)
        y += ROW_SPACING
    route.append((ROOM_X[0], ys[0]))
    east = True
    for y in ys:
        x_start, x_end = (ROOM_X if east else ROOM_X[::-1])
        if in_band(y):
            # split the row at the zone and detour around it through a clear row
            clear_y = RED["y"][0] - 1.5 if y < ZONE[1] else RED["y"][1] + 1.5
            near, far = (RED["x"][0], RED["x"][1]) if east else (RED["x"][1], RED["x"][0])
            route += [(x_start, y), (near, y), (near, clear_y),
                      (far, clear_y), (far, y), (x_end, y)]
        else:
            route += [(x_start, y), (x_end, y)]
        east = not east
    return route


class Recorder:
    """Write the camera feed to an MP4 from a background thread.

    Runs its own node and executor so the flight loop never starves it.
    Frames are written at a fixed rate (wall clock), so playback is real time.
    """

    def __init__(self, path, topic="/camera/image_raw", fps=15.0):
        self.path, self.fps = path, fps
        self.node = Node("mission_recorder")
        self.bridge = CvBridge()
        self.writer, self.frames = None, 0
        self.t0, self._last = None, 0.0
        self._lock = threading.Lock()
        self._stopped = False
        self.node.create_subscription(
            Image, topic, self._on_image,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        atexit.register(self.stop)
        print(f"Recording {topic} -> {path}")

    def _on_image(self, msg):
        now = time.time()
        with self._lock:
            if now - self._last < 1.0 / self.fps:
                return
            self._last = now
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:  # noqa: BLE001 - skip a bad frame, keep recording
                return
            if self.writer is None:
                h, w = frame.shape[:2]
                self.writer = cv2.VideoWriter(
                    self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
                if not self.writer.isOpened():
                    print(f"WARNING: cannot open video writer for {self.path}")
                    self.writer = False
                    return
                self.t0 = now
            if not self.writer:
                return
            cv2.putText(frame, f"t={now - self.t0:6.1f}s", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            self.writer.write(frame)
            self.frames += 1

    def stop(self):
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            if self.writer:                      # an open VideoWriter
                self.writer.release()
                print(f"Recording saved: {self.path} ({self.frames} frames, "
                      f"{self.frames / self.fps:.0f} s)")
            elif self.frames == 0:
                print("Recording stopped: no frames were received on the camera topic.")
        try:                                     # stop the background thread cleanly
            self.executor.shutdown()
            self.thread.join(timeout=2.0)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001 - rclpy may already be shut down at exit
            pass


class Drone:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url)
        self.m.wait_heartbeat()
        self.sys, self.comp = self.m.target_system, self.m.target_component
        self.ned = (0.0, 0.0, 0.0)
        self.yaw = 0.0                      # rad, NED (0 = north, +pi/2 = east)
        self._last_hb = 0.0

    def poll(self):
        """Drain incoming messages, keep our GCS heartbeat alive."""
        now = time.time()
        if now - self._last_hb > 1.0:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_hb = now
        while True:
            msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg is None:
                break
            self.ned = (msg.x, msg.y, msg.z)
        while True:
            msg = self.m.recv_match(type="ATTITUDE", blocking=False)
            if msg is None:
                break
            self.yaw = msg.yaw

    def setpoint(self, n, e, d):
        # position only: ignore velocity, accel, yaw, yaw rate
        self.m.mav.set_position_target_local_ned_send(
            0, self.sys, self.comp, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000, n, e, d, 0, 0, 0, 0, 0, 0, 0, 0)

    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.sys, self.comp, command, 0, *p)

    def arm(self, force):
        self.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196 if force else 0)

    def offboard(self):
        self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 6, 0)

    def land(self):
        self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4, 6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--alt", type=float, default=3.0)
    ap.add_argument("--ref-alt", type=float, default=2.0,
                    help="altitude while reading the start QR")
    ap.add_argument("--speed", type=float, default=1.5, help="m/s along the route")
    ap.add_argument("--force-arm", action="store_true")
    ap.add_argument("--record", default="", metavar="FILE.mp4",
                    help="save the down-camera feed to this file until the drone "
                         "is centred over the QR (off by default)")
    ap.add_argument("--record-fps", type=float, default=15.0)
    ap.add_argument("--record-topic", default="/red_zone_node/debug_image",
                    help="feed to record: the red-zone-annotated image (default) "
                         "or /camera/image_raw for the raw camera")
    ap.add_argument("--confirm", type=int, default=2, help="consecutive matched=True needed")
    ap.add_argument("--img-w", type=int, default=1280, help="down camera width, px")
    ap.add_argument("--img-h", type=int, default=960, help="down camera height, px")
    ap.add_argument("--hfov", type=float, default=1.74, help="down camera horizontal FOV, rad")
    ap.add_argument("--center-tol", type=float, default=20.0,
                    help="pixel error at which the drone counts as centred on the QR")
    ap.add_argument("--center-gain", type=float, default=0.4,
                    help="fraction of the measured offset corrected per step")
    ap.add_argument("--max-step", type=float, default=0.5,
                    help="largest single correction, metres")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds to dwell after arriving before trusting a result")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("lawnmower_mission")
    state = {"hits": 0, "ref_ready": False, "east": 0.0, "last": None, "n": 0}

    def on_matched(msg):
        # the start QR matches itself, so ignore results until inside the room
        if msg.data and state["east"] >= MATCH_MIN_X:
            state["hits"] += 1
        else:
            state["hits"] = 0

    def on_result(msg):
        state["last"] = msg
        state["n"] += 1

    def on_ref_ready(msg):
        state["ref_ready"] = msg.data
    # qr_match publishes latched (TRANSIENT_LOCAL) results; default sub QoS is fine
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, "/qr_match_node/matched", on_matched, qos)
    node.create_subscription(Bool, "/qr_match_node/reference_ready", on_ref_ready, qos)
    node.create_subscription(QRResult, "/qr_match_node/result", on_result, qos)
    capture = node.create_client(Trigger, "/qr_match_node/capture_reference")

    rec = (Recorder(args.record, topic=args.record_topic, fps=args.record_fps)
           if args.record else None)

    def stop_recording():
        if rec is not None:
            rec.stop()

    d = Drone(args.url)
    print("Connected to PX4. Streaming setpoints...")
    hold = (0.0, 0.0, 0.0)
    for _ in range(40):                       # >1 s of setpoints before offboard
        d.poll()
        d.setpoint(*hold)
        time.sleep(0.05)
    d.offboard()
    time.sleep(0.3)
    d.arm(args.force_arm)

    # wait for the vehicle to leave the ground state
    t0 = time.time()
    while True:
        d.poll()
        d.setpoint(*hold)
        rclpy.spin_once(node, timeout_sec=0)
        if -d.ned[2] > 0.0 or time.time() - t0 > 3:
            break
        time.sleep(0.05)
    print("Armed (if it does not lift off, re-run with --force-arm).")

    # climb to cruise altitude above the pad
    target = world_to_ned(0, PAD_Y, args.alt)
    while abs(d.ned[2] - target[2]) > 0.3:
        d.poll()
        d.setpoint(*target)
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.05)
    print(f"At {args.alt} m.")

    # ---- learn the reference: hover over the start QR and capture it ----
    ref = world_to_ned(*START_QR, args.ref_alt)
    cur = list(d.ned)
    while math.dist(d.ned, ref) > 0.4:
        d.poll()
        rclpy.spin_once(node, timeout_sec=0)
        dist = math.dist(cur, ref)
        step = min(args.speed * 0.05, dist)
        if dist > 1e-6:
            cur = [c + (t - c) * step / dist for c, t in zip(cur, ref)]
        d.setpoint(*cur)
        time.sleep(0.05)
    print("Over the start QR. Capturing reference...")
    if not capture.wait_for_service(timeout_sec=10.0):
        raise SystemExit("qr_match_node not running (no capture_reference service).")
    state["ref_ready"] = False
    future = capture.call_async(Trigger.Request())
    t0 = time.time()
    while not state["ref_ready"]:
        d.poll()
        d.setpoint(*ref)                 # keep hovering while the node looks
        rclpy.spin_once(node, timeout_sec=0)
        if time.time() - t0 > 15:
            raise SystemExit("Reference not captured within 15 s: no QR seen "
                             "by the camera. Check /camera/image_raw.")
        time.sleep(0.05)
    print("Reference captured. Starting route.")

    route = build_route()
    cur = list(d.ned)
    dt = 0.05
    found = None
    for i, (wx, wy) in enumerate(route):
        tgt = world_to_ned(wx, wy, args.alt)
        while True:
            d.poll()
            state["east"] = d.ned[1]
            rclpy.spin_once(node, timeout_sec=0)
            if state["hits"] >= args.confirm:
                found = d.ned
                break
            dist = math.dist(cur, tgt)
            if dist < 0.4 and math.dist(d.ned[:2], tgt[:2]) < 0.8:
                break
            step = min(args.speed * dt, dist)         # slew the setpoint along the segment
            if dist > 1e-6:
                cur = [c + (t - c) * step / dist for c, t in zip(cur, tgt)]
            d.setpoint(*cur)
            time.sleep(dt)
        if found:
            break
        print(f"waypoint {i + 1}/{len(route)} reached: world ({wx:.1f}, {wy:.1f})")

    if not found:
        stop_recording()
        print("Sweep finished without a match.")
    else:
        wx, wy = found[1], found[0] + PAD_Y
        near = min(KNOWN_QR.items(), key=lambda kv: math.dist((wx, wy), kv[1]))
        last = state["last"]
        if last is not None:
            for det in last.detections:
                print(f"  matched on: text={det.decoded_text!r} score={det.score:.3f} "
                      f"from_text={det.score_from_decoded_text} matched={det.matched}")
        print(f"  drone local NED = ({found[0]:.2f}, {found[1]:.2f}, {found[2]:.2f})")
        print(f"MATCH! drone over world ({wx:.1f}, {wy:.1f}); "
              f"closest tile is {near[0]} at {near[1]}")
        hold = [found[0], found[1], found[2]]

        # ---- centre over the QR using its position in the down-camera image ----
        # Image up = drone forward, image right = drone right (camera pitched
        # 90 deg down). Pixel -> metres uses the current height above ground.
        print("Centring over the QR...")
        cx, cy = args.img_w / 2.0, args.img_h / 2.0
        print(f"  drone yaw = {math.degrees(d.yaw):.0f} deg (NED; 90 = facing east)")
        centred, t0 = 0, time.time()
        moved_at, n_gate = time.time(), state["n"] + 99   # n_gate: first result index we may trust
        while centred < 3 and time.time() - t0 < 60:
            d.poll()
            d.setpoint(*hold)
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(0.05)
            # Results lag the camera by the detector's processing time, so only
            # trust one that began after the drone arrived at the last setpoint
            # and settled: wait for arrival, dwell, then skip results in flight.
            if math.dist(d.ned[:2], hold[:2]) > 0.15:
                moved_at, n_gate = time.time(), state["n"] + 99
                continue
            if time.time() - moved_at < args.settle:
                n_gate = state["n"] + 2          # skip the 2 results still in flight
                continue
            if state["n"] < n_gate:
                continue
            n_gate = state["n"] + 99
            res = state["last"]
            dets = [x for x in res.detections if x.matched] if res else []
            if not dets:
                n_gate = state["n"] + 1          # QR lost this frame: try the next one
                continue
            x1, y1, x2, y2 = dets[0].bbox_xyxy
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            metres_per_px = 2.0 * max(-d.ned[2], 0.5) * math.tan(args.hfov / 2.0) / args.img_w
            fwd, right = -(v - cy) * metres_per_px, (u - cx) * metres_per_px
            err_px = math.hypot(u - cx, v - cy)
            if err_px <= args.center_tol:
                centred += 1
                moved_at = time.time()           # keep sampling while stationary
                n_gate = state["n"] + 2
                continue
            centred = 0
            g = args.center_gain
            step_f, step_r = g * fwd, g * right
            norm = math.hypot(step_f, step_r)
            if norm > args.max_step:             # cap each correction
                step_f, step_r = step_f * args.max_step / norm, step_r * args.max_step / norm
            hold[0] += step_f * math.cos(d.yaw) - step_r * math.sin(d.yaw)
            hold[1] += step_f * math.sin(d.yaw) + step_r * math.cos(d.yaw)
            moved_at = time.time()
            print(f"  offset {err_px:.0f} px (u={u:.0f}, v={v:.0f}) -> "
                  f"move fwd {step_f:+.2f} m, right {step_r:+.2f} m")
        stop_recording()                       # feed is saved up to this point
        if centred >= 3:
            print("Centred over the QR.")
        else:
            print("Could not fully centre (QR lost or still drifting); holding here.")
        wx, wy = hold[1], hold[0] + PAD_Y
        print(f"Holding over world ({wx:.2f}, {wy:.2f}); closest tile is "
              f"{min(KNOWN_QR.items(), key=lambda kv: math.dist((wx, wy), kv[1]))[0]}. "
              "Ctrl+C to land.")
        try:
            while True:                        # payload system goes here later
                d.poll()
                d.setpoint(*hold)
                rclpy.spin_once(node, timeout_sec=0)
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
    d.land()
    print("Landing.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
