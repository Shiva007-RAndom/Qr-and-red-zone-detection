#!/usr/bin/env python3
"""ROS 2 node: detect red (restricted-zone) patches in an image stream.

Runs alongside qr_match_node on the same camera topic.

Subscribes
    <image_topic>   sensor_msgs/Image (or CompressedImage)

Publishes
    ~/detected     std_msgs/Bool                    any red patch in the frame
    ~/result       qr_match_interfaces/RedZoneResult  per-patch detail
    ~/debug_image  sensor_msgs/Image                annotated frame (optional)

Detection (ported from the red_zone_detection notebook):
    image -> HSV -> two red hue ranges (red wraps the hue wheel) -> open/close
          -> external contours -> keep contours with area >= min_area
          -> centroid from image moments.
This node only reports where the red is; avoiding it comes later.
"""

from __future__ import annotations

import time
from typing import List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

from qr_match_interfaces.msg import RedPatch, RedZoneResult


# ======================================================================
# Detection (plain functions -- no ROS, no shared state)
# ======================================================================
def red_mask(bgr, lower1, upper1, lower2, upper2, kernel_size: int = 7) -> np.ndarray:
    """Binary mask of red pixels, cleaned with open + close."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1),
                          cv2.inRange(hsv, lower2, upper2))
    k = max(3, int(kernel_size) | 1)  # force odd
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def find_patches(mask, min_area: float) -> List[dict]:
    """Contours of the mask with area >= min_area, largest first."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    patches = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        patches.append({
            "contour": cnt,
            "area": float(area),
            "centroid": (m["m10"] / m["m00"], m["m01"] / m["m00"]),
            "bbox": cv2.boundingRect(cnt),
        })
    patches.sort(key=lambda p: p["area"], reverse=True)
    return patches


def annotate(bgr, patches, elapsed_ms: float) -> np.ndarray:
    """Draw filled translucent contours, centroids and a summary line."""
    out = bgr.copy()
    for p in patches:
        overlay = out.copy()
        cv2.drawContours(overlay, [p["contour"]], -1, (0, 255, 0), cv2.FILLED)
        cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)
        cv2.drawContours(out, [p["contour"]], -1, (0, 255, 0), 2)
        cx, cy = int(p["centroid"][0]), int(p["centroid"][1])
        cv2.circle(out, (cx, cy), 5, (255, 0, 0), -1)
        cv2.putText(out, f"Red ({int(p['area'])}px2)", (cx + 8, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    cv2.putText(out, f"Red patches: {len(patches)}  |  {elapsed_ms:.1f} ms",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return out


# ======================================================================
# ROS 2 node
# ======================================================================
class RedZoneNode(Node):
    def __init__(self) -> None:
        super().__init__("red_zone_node")

        # ---- parameters ----
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("best_effort_qos", True)
        self.declare_parameter("min_area", 100)
        self.declare_parameter("hsv_lower1", [0, 80, 40])
        self.declare_parameter("hsv_upper1", [15, 255, 255])
        self.declare_parameter("hsv_lower2", [165, 80, 40])
        self.declare_parameter("hsv_upper2", [180, 255, 255])
        self.declare_parameter("kernel_size", 7)
        self.declare_parameter("contour_epsilon", 0.01)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("log_every_frame", False)

        def hsv(name) -> np.ndarray:
            return np.array(self.get_parameter(name).value, dtype=np.uint8)

        image_topic = self.get_parameter("image_topic").value
        self.use_compressed = bool(self.get_parameter("use_compressed").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.lower1, self.upper1 = hsv("hsv_lower1"), hsv("hsv_upper1")
        self.lower2, self.upper2 = hsv("hsv_lower2"), hsv("hsv_upper2")
        self.kernel_size = int(self.get_parameter("kernel_size").value)
        self.epsilon = float(self.get_parameter("contour_epsilon").value)
        self.every_n = max(1, int(self.get_parameter("process_every_n").value))
        self.debug = bool(self.get_parameter("publish_debug_image").value)
        self.log_every = bool(self.get_parameter("log_every_frame").value)

        self.bridge = CvBridge()
        self.frame_count = 0

        # ---- QoS: same convention as qr_match_node ----
        rel = (QoSReliabilityPolicy.BEST_EFFORT
               if bool(self.get_parameter("best_effort_qos").value)
               else QoSReliabilityPolicy.RELIABLE)
        image_qos = QoSProfile(reliability=rel,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        result_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ---- pub/sub ----
        msg_type = CompressedImage if self.use_compressed else Image
        self.create_subscription(msg_type, image_topic, self.on_image, image_qos)
        self.pub_detected = self.create_publisher(Bool, "~/detected", result_qos)
        self.pub_result = self.create_publisher(RedZoneResult, "~/result", result_qos)
        self.pub_debug = (self.create_publisher(Image, "~/debug_image", image_qos)
                          if self.debug else None)

        self.get_logger().info(
            f"Ready. Listening on '{image_topic}', min_area {self.min_area:.0f} px2, "
            f"1 in {self.every_n} frame(s).")

    def _contour_points(self, cnt) -> List[Point32]:
        eps = self.epsilon * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, eps, True).reshape(-1, 2)
        return [Point32(x=float(x), y=float(y), z=0.0) for x, y in approx]

    def on_image(self, msg) -> None:
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return
        try:
            if self.use_compressed:
                frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return
        if frame is None or frame.size == 0:
            return

        start = time.perf_counter()
        try:
            mask = red_mask(frame, self.lower1, self.upper1,
                            self.lower2, self.upper2, self.kernel_size)
            patches = find_patches(mask, self.min_area)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.get_logger().error(f"Processing failed on this frame: {exc}")
            return
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        height, width = frame.shape[:2]
        image_area = float(width * height)

        result = RedZoneResult()
        result.header = msg.header
        result.image_width = int(width)
        result.image_height = int(height)
        result.detected = len(patches) > 0
        result.num_patches = len(patches)
        result.total_area_fraction = float(sum(p["area"] for p in patches) / image_area)
        result.processing_time_ms = float(elapsed_ms)
        for p in patches:
            cx, cy = p["centroid"]
            rp = RedPatch()
            rp.centroid_x, rp.centroid_y = float(cx), float(cy)
            rp.offset_x = float((cx - width / 2.0) / (width / 2.0))
            rp.offset_y = float((cy - height / 2.0) / (height / 2.0))
            rp.area_px = p["area"]
            rp.area_fraction = float(p["area"] / image_area)
            rp.bbox_xywh = [int(v) for v in p["bbox"]]
            rp.contour = self._contour_points(p["contour"])
            result.patches.append(rp)
        self.pub_result.publish(result)
        self.pub_detected.publish(Bool(data=result.detected))

        if self.pub_debug is not None:
            try:
                dbg = self.bridge.cv2_to_imgmsg(annotate(frame, patches, elapsed_ms), "bgr8")
                dbg.header = msg.header
                self.pub_debug.publish(dbg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"debug image failed: {exc}")

        if self.log_every or result.detected:
            if result.detected:
                top = result.patches[0]
                self.get_logger().info(
                    f"RED: {len(patches)} patch(es), largest {top.area_px:.0f} px2 "
                    f"at ({top.centroid_x:.0f}, {top.centroid_y:.0f}) "
                    f"({elapsed_ms:.0f} ms)", throttle_duration_sec=1.0)
            else:
                self.get_logger().info(f"no red ({elapsed_ms:.0f} ms)")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RedZoneNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"red_zone_node terminated: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
