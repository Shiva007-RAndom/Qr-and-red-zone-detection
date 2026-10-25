# AEROTHON 2026 — Mission 2 "SkyScan" (Gazebo + PX4 + ROS 2)

Simulation of **Flight Mission 2 – SkyScan: Autonomous Rapid Delivery**
(rulebook Section 4.2.4, Figure 3): a Gazebo world, a PX4 quadrotor with a
downward camera, and the autonomy pieces that run on it.

- [1. What this does and how to run it](#1-what-this-does-and-how-to-run-it)
- [2. What is still to be built](#2-what-is-still-to-be-built) — **red-zone avoidance** and **payload drop**
- [3. Repository documentation](#3-repository-documentation)

---

## 1. What this does and how to run it

### What happens in a run

The drone takes off from the launch pad and does this on its own:

1. **Learns the reference.** It hovers over the **start QR** (2 m ahead of the
   pad) and memorises it from the camera.
2. **Sweeps the room.** It flies the corridor, then a lawnmower pattern over
   the 40 × 40 m room at 3 m altitude, going around the red restricted zone.
3. **Finds the match.** Four candidate QR codes lie in the room; only one is
   identical to the start QR. The sweep stops when the QR matcher reports a
   match.
4. **Centres and holds.** It steers onto the matched QR using the QR's position
   in the camera image, then hovers directly above it. (It does not land and
   does not drop anything yet — see part 2.)
5. **Optional:** the camera feed, with the red zone outlined, is saved to an MP4
   up to this point.

Two detectors run on the downward camera at the same time:

| Node | Job |
|---|---|
| `qr_match_node` | finds QR codes, compares each to the reference, says which match |
| `red_zone_node` | finds red patches (the restricted zone) and where they are in the image |

### Prerequisites

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`gz sim` 8), PX4-Autopilot built for
SITL (`~/PX4-Autopilot`, `build/px4_sitl_default` exists), plus:

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-cv-bridge ros-jazzy-rqt-image-view \
                 python3-colcon-common-extensions libzbar0
pip3 install pymavlink                    # system Python: used by the mission script
```

The QR matcher needs a Python venv with `qrdet`/`torch` — follow
`QR_Node/ros2_ws/src/qr_match/RUN_AND_DEPLOY.md` §1b (it creates
`~/qr_match_venv`). `qrdet` downloads its weights on first run (needs internet
once).

### One-time setup

```bash
# Set REPO permanently so it is available in every new terminal (adjust to your clone)
echo 'export REPO="$HOME/ML/aerothon_mission2_gazebo_world_v6"' >> ~/.bashrc
source ~/.bashrc

# 1. Build the ROS workspace (qr_match, red_zone, and their messages)
source /opt/ros/jazzy/setup.bash
cd $REPO/QR_Node/ros2_ws && colcon build --packages-select qr_match_interfaces qr_match red_zone

# 2. Install the world and models where Gazebo / PX4 look for them
mkdir -p ~/.simulation-gazebo/models
for m in $REPO/models/*; do rm -rf ~/.simulation-gazebo/models/$(basename $m); cp -r $m ~/.simulation-gazebo/models/; done
cp $REPO/worlds/aerothon_mission2.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/      # PX4 only finds worlds in its own folder
cp $REPO/models/x500_mono_cam_down/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf
```

Redo step 2 whenever you change anything under `models/` or `worlds/`, then
**fully restart Gazebo** — textures and worlds are only loaded at start.

### Run it (4 terminals, in this order)

**Terminal 1 — Gazebo + PX4**
```bash
pkill -x px4                                   # clears an old PX4 if one is still running
source /opt/ros/jazzy/setup.bash
export GZ_CONFIG_PATH="/usr/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz"
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/.simulation-gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
cd ~/PX4-Autopilot
PX4_GZ_WORLD=aerothon_mission2 PX4_GZ_MODEL_POSE="0,20,0.1,0,0,0" make px4_sitl gz_x500_mono_cam_down
```
The first time only, type this at the `pxh>` prompt (it is saved):
```
param set NAV_DLL_ACT 0
param set COM_RCL_EXCEPT 4
param save
```
Without it PX4 refuses to arm with `Preflight Fail: No connection to the GCS`.

**Terminal 2 — camera bridge + both detectors**
```bash
source /opt/ros/jazzy/setup.bash
source ~/qr_match_venv/bin/activate
source "$REPO/QR_Node/ros2_ws/install/setup.bash"
ros2 launch qr_match skyscan_qr.launch.py
```
Wait for `Ready. Listening on '/camera/image_raw'` from `qr_match_node` (the YOLO
model takes a moment to load).

**Terminal 3 — the mission** (system Python, *not* the venv)
```bash
source /opt/ros/jazzy/setup.bash
source $REPO/QR_Node/ros2_ws/install/setup.bash
python3 $REPO/launch/lawnmower_mission.py --record ~/skyscan_run.mp4
```
Drop `--record ...` if you don't want the video. Add `--force-arm` if the drone
won't arm.

**Terminal 4 (optional) — watch**
```bash
source /opt/ros/jazzy/setup.bash
source $REPO/QR_Node/ros2_ws/install/setup.bash
ros2 topic echo /qr_match_node/matched
ros2 topic echo /qr_match_node/result --field detections --flow-style   # score + decoded text per QR
ros2 run rqt_image_view rqt_image_view /red_zone_node/debug_image       # red-zone outline on the live feed
```

### What success looks like

```
Over the start QR. Capturing reference...
Reference captured. Starting route.
...
MATCH! drone over world (18.0, 7.6); closest tile is qr_b at (18, 8)
Centring over the QR...
  offset 80 px ... -> move fwd ..., right ...
Centred over the QR.
Recording saved: ~/skyscan_run.mp4 (N frames, S s)
Holding over world (17.93, 7.93); closest tile is qr_b. Ctrl+C to land.
```
Order of things the drone passes: `qr_c`, `qr_d`, **part of the red zone**, then
`qr_b` (the correct one). `qr_c` and `qr_d` must **not** match.
`Ctrl+C` lands the drone.

Mission script options (`python3 lawnmower_mission.py --help`): `--alt` (cruise
height, 3 m), `--speed` (1.5 m/s), `--record FILE.mp4`, `--record-topic`
(default is the red-zone-annotated feed; use `/camera/image_raw` for the raw
one), `--force-arm`, and the centring tuning `--center-tol / --center-gain /
--max-step / --settle`.

---

## 2. What is still to be built

Both of these start from the point where the drone is hovering over the correct
QR, or from the sweep itself. Everything they need to read is already
published; nothing consumes it yet.

### A. Red-zone avoidance

**Where we are.** The drone already avoids the zone — but only because the
route is **precomputed from hard-coded geometry**. In
`launch/lawnmower_mission.py`:

- `ZONE = (33.0, 14.0)` and `RED` (the zone plus a 1.5 m margin) are constants.
- `build_route()` splits any sweep row that would cross the zone and detours
  around it.

So move the red zone in the world file and the drone will fly straight over it.
`red_zone_node` already detects the zone live from the camera, but **nothing
uses its output**. The job is to replace the hard-coded keep-out box with
what the camera actually sees.

**What you get to work with** — `/red_zone_node/result`
(`qr_match_interfaces/RedZoneResult`, latched, ~15 Hz):

| Field | Meaning |
|---|---|
| `detected`, `num_patches`, `total_area_fraction` | any red in the frame, how much |
| `patches[]` (largest first) | one entry per red region |
| `patches[i].centroid_x/y`, `offset_x/y` | centroid in pixels, and offset from image centre in [-1, 1] |
| `patches[i].area_px`, `area_fraction`, `bbox_xywh` | size |
| `patches[i].contour` | simplified outline polygon in pixel coordinates |
| `header` | same stamp as the camera frame |

`/red_zone_node/detected` (`Bool`) is the quick yes/no.
`/red_zone_node/debug_image` shows what it sees.

**What needs doing**
1. **Pixels → world.** Project a patch's contour to world coordinates, using the
   drone's position, yaw and height. The QR centring code in
   `lawnmower_mission.py` already does the same conversion for one point
   (`metres_per_px = 2 * height * tan(hfov/2) / img_w`; image *up* = drone
   forward, image *right* = drone right; rotate by yaw into NED). Reuse it for
   the contour.
2. **Remember the zone.** The camera only sees ±3.6 m (left–right) and ±2.4 m
   (forward–back) from 3 m up, and usually only part of the zone. Accumulate
   what is seen into a world-frame polygon/occupancy map, inflated by a safety
   margin.
3. **Re-plan the remaining route** so no segment enters the inflated zone, while
   still covering the room (the existing row-splitting detour in
   `build_route()` is a starting point). Do it while flying, not up front.
4. **Handle the edge cases:** zone seen only at the edge of the frame; zone
   between the drone and the next waypoint; a QR tile close to the zone.

**Done when** the drone completes the sweep and finds the QR with the zone moved
to a different spot in `worlds/aerothon_mission2.sdf` (the `red_zone_1` pose),
with the hard-coded `ZONE` removed, and never crosses the boundary (log the
closest approach). Geo-fence coordinates are "to be provided in Phase 2" in the
rulebook; nothing in the repo encodes a fence.

**Watch out for:** the HSV thresholds are in `red_zone/config/red_zone.yaml`.
The QR tile borders are deliberately **not** red (blue / teal / purple) so the
detector does not fire on them — keep it that way if you add tiles. Detection
takes ~7 ms per frame, so latency is not the issue; the drone's field of view is.

### B. Payload drop

**Where we are.** After the drone is centred over the matched QR it just
**hovers**. The last loop in `main()` of `launch/lawnmower_mission.py`
(`while True: ... # payload system goes here later`) streams the hold position
until you press Ctrl+C. There is **no payload in the simulation**: the drone
model (`x500_mono_cam_down`) has no gripper or release mechanism, and no code
drops anything.

**What needs doing**
1. **Add a payload to the simulation.** Pick a mechanism and model it — for
   example a small box attached to the drone with Gazebo's `DetachableJoint`
   system, released by publishing to a Gazebo topic (or a spawned model dropped
   at release). It must be added to the drone model in `models/`, installed
   with the one-time setup, and needs a way to release from the script.
2. **Choose a drop height and descend to it** while staying centred. The
   centring loop already works at any height (the pixel-to-metre scale uses the
   current altitude) — check the QR stays large enough to detect when low.
3. **Release**, then **verify**: read the payload's position from Gazebo
   (`/world/aerothon_mission2/pose/info` or `gz topic -e`) and check it landed
   within a set distance of the QR centre (≈ 0.8 m tile). Report the result.
4. **Climb back to cruise height** and hand over to whatever comes next.

**Done when** a run ends with the payload on the correct QR (distance printed),
the drone back at cruise height, and nothing dropped on a wrong tile if the
match is lost.

**Hooks:** the centred position is `hold` (a `[north, east, down]` list in
local NED); `state["last"]` holds the latest `QRResult` (use `detections[0]`
for the matched QR's pixel box); `d.setpoint(n, e, d)` moves the drone,
`d.land()` lands it.

### Also not done (not asked for, but missing)

- The **return trip**: fly back through the return lane (it already has two box
  obstacles at (6, 17.3) and (10, 19.4)) and land on the pad.
- The drone has **only the downward camera**. A front camera is not modelled.

---

## 3. Repository documentation

### Layout

```
.
├── README.md
├── launch/
│   ├── skyscan_qr.launch.py       camera bridge + red_zone_node + qr_match_node
│   └── lawnmower_mission.py       the mission: takeoff, read reference, sweep, centre, hold, (record)
├── worlds/aerothon_mission2.sdf   the Gazebo world (no vehicle in it; PX4 spawns the drone)
├── models/                        custom models + PX4's x500 files (see below)
├── QR_Node/                       its own git repo — the ROS 2 workspace
│   └── ros2_ws/src/
│       ├── qr_match/              QR detect + match node
│       ├── red_zone/              red-zone detection node
│       └── qr_match_interfaces/   all custom messages
├── red_zone_detection (1).ipynb   the original red-zone notebook the node was ported from
└── px4_gazebo_camera_fix (1).md   older camera troubleshooting notes (see "Gotchas")
```

`QR_Node/` is a separate git repository; commit changes there separately.

### The world

World origin `(0, 0)` = launch pad centre, floor level. Gazebo is X east, Y north,
Z up. World file marker: `VERSION MARKER: v6-2026-09-30`.

```
launch pad --2m--> start QR --1m--> [ 10 m x 7 m two-lane corridor ] --> ROOM (40 x 40, walled, open top)
```

| Thing | Where |
|---|---|
| Launch pad / drone spawn | `(0, 20)`, drone faces +X |
| Start QR (the reference) | `(2, 20)` |
| Corridor | X 3–13. `lane_forward` Y 20–23.5 (clear), `lane_return` Y 16.5–20 (obstacles at `(6, 17.3)` and `(10, 19.4)`, 0.5 × 0.5 × 1.2 m) |
| Entrance banners | X = 3 and X = 13, panels at Z 4.2–5.8 m (fly below 4 m) |
| Room | X 13–53, Y 0–40, 2.5 m walls, 7 m gap in the west wall at Y 16.5–23.5 |
| **Red zone** | **centre `(33, 14)`, 8 × 8 m**, flat, no collision (a no-fly zone for the autonomy, not a wall) |
| Candidate QRs (0.8 m tiles) | `qr_a (20, 32)`, `qr_b (18, 8)`, `qr_c (24, 4)`, `qr_d (46, 7.5)` |

The **correct** QR is `qr_b`: its image is identical to `qr_start`. Sweep order
puts it third among the QRs, with part of the red zone seen before it
(`qr_c`, `qr_d`, red zone, `qr_b`), so a matcher that wrongly matches the first
QR it sees fails the test.

### Models (`models/`)

| Model | What |
|---|---|
| `qr_start`, `qr_a`–`qr_d` | flat 0.8 m QR tiles. `qr_e` exists but is not placed in the world |
| `takeoff_pad` | launch marker, 3 × 7.2 m |
| `banner_forward`, `banner_return` | corridor entrance banners |
| `black_obstacle` | return-lane box obstacle |
| `red_zone` | the flat red restricted zone, colour `0.85 0.1 0.1` |
| `x500_mono_cam_down` | PX4's x500 + downward camera (1280 × 960, 30 Hz, FOV 1.74 rad ≈ 100°) |
| `x500`, `x500_base`, `mono_cam` | PX4's stock files, dependencies of the drone |

QR payloads (real, scannable): `AEROTHON2026|TARGET|ID:QR_A` … `QR_D`; `qr_start`
and `qr_b` both decode to `…ID:QR_B`. Tile border colours: `qr_b`/`qr_start`
**blue**, `qr_a`/`qr_d` teal, `qr_c` purple — none red, on purpose.

### The ROS 2 pipeline

```
Gazebo camera ──ros_gz_bridge──> /camera/image_raw ──┬──> qr_match_node ──> /qr_match_node/{detected,matched,result,reference_ready}
 (x500_mono_cam_down_0/.../camera/image)             └──> red_zone_node ──> /red_zone_node/{detected,result,debug_image}
                                                                                        │
PX4 (MAVLink, udp 14540) <── lawnmower_mission.py <──────────────────────────────────────┘ (reads results, flies)
```

**`qr_match_node`** (`qr_match/qr_match/qr_match_node.py`)
1. `qrdet` (YOLO) finds QR codes. 2. Each is perspective-warped flat.
3. Binarised and sampled into an N × N module grid. 4. If both it and the
reference decode to text, the text is compared exactly; otherwise the grid
similarity over 4 rotations is the score. 5. `matched` = score ≥ `match_threshold` (0.85).

The reference comes from the camera: call the service `~/capture_reference`
(`std_srvs/Trigger`); the next frame's best QR becomes the reference and
`~/reference_ready` goes true. (`reference_qr_path` still works to preload one.)
Until a reference exists nothing matches. Messages: `QRResult`, `QRDetection`.
More detail: `QR_Node/ros2_ws/src/qr_match/README.md`.

**`red_zone_node`** (`red_zone/red_zone/red_zone_node.py`)
HSV threshold with two red ranges → open/close → contours → keep area ≥
`min_area` (100 px²) → centroids from image moments. Publishes
`RedZoneResult` / `RedPatch` (see part 2A) and an annotated `~/debug_image`.
Ported from the notebook's last cell. Parameters: `red_zone/config/red_zone.yaml`.

**Messages** (all in `qr_match_interfaces`): `QRResult`, `QRDetection`,
`RedZoneResult`, `RedPatch`.

**`launch/skyscan_qr.launch.py`** starts the camera bridge (Gazebo topic
`/world/aerothon_mission2/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`
→ `/camera/image_raw`; the `_0` is added by PX4), `red_zone_node`
(every 2nd frame) and `qr_match_node` (every 3rd frame). Detection is CPU-bound;
raise `process_every_n` if it can't keep up.

### The mission script (`launch/lawnmower_mission.py`)

Uses `pymavlink` on UDP 14540 in **offboard** mode, plus ROS 2 for the detector
topics.

1. Streams setpoints, switches to offboard, arms, climbs to `--alt`.
2. Flies to the start QR at `--ref-alt` (2 m), calls `capture_reference`, waits
   for `reference_ready`.
3. `build_route()`: corridor (forward lane) → serpentine over the room, 8 rows 5 m
   apart (`ROW_SPACING`), rows crossing the red zone are split and detoured.
4. Flies the route by sliding the setpoint at `--speed`. Stops after `--confirm`
   (2) consecutive `matched = true` — **ignored while X < 13.5 m**, because the
   start QR matches itself.
5. Centring loop: takes the matched QR's pixel box, converts the offset from
   image centre to metres, moves the hold point by `--center-gain` (0.4) of it,
   capped at `--max-step`, waits for the drone to arrive and settle, skips the
   results still in flight, repeats until 3 readings are within `--center-tol`.
6. Stops the recording, then hovers until Ctrl+C (then lands).

**Frames.** PX4 local NED is anchored at the spawn point: `north = Y_world − 20`,
`east = X_world`, `down = −altitude`. The camera is pitched 90° down: image *up* =
drone forward, image *right* = drone right.

### Gotchas (learned the hard way)

- **PX4 only finds worlds in its own folder.** Copy the world into
  `~/PX4-Autopilot/Tools/simulation/gz/worlds/`, or you get
  `Waiting for Gazebo world...` forever.
- **Texture names must be unique across models.** Every QR/banner texture has its
  own filename (`qr_b_tile.png`, …). Sharing a name makes Gazebo show one image
  on several models, and the QR matcher then "finds" the wrong tile.
- **Restart Gazebo after changing models/worlds.** Textures load at start only.
- **Camera orientation.** The 90° pitch lives on the *include* pose in
  `models/x500_mono_cam_down/model.sdf`; the `CameraJoint` pose is zero. The
  older `px4_gazebo_camera_fix (1).md` suggests moving the pitch onto the joint —
  that points the camera **forward**, don't. The fixed file must also be in
  PX4's own models folder (setup step 2), because PX4's folder is searched first.
- **Arming:** needs the two PX4 params in part 1 (or `--force-arm`).
- **Don't use `pkill -f "gz sim"`** — it matches its own shell and kills your
  terminal. Use `pkill -x px4`; close Gazebo with Ctrl+C.
- The mission script must run with **system Python** (it needs `pymavlink` and
  `rclpy`); only `qr_match_node` needs the `qr_match_venv`.
- If you move `QR_Node/`, delete `ros2_ws/build install log` and rebuild.
