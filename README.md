# AEROTHON 2026 — Mission 2 "SkyScan" (Gazebo + PX4 + ROS 2)

Simulation of **Flight Mission 2 – SkyScan: Autonomous Rapid Delivery**
(rulebook Section 4.2.4, Figure 3): a Gazebo world, a PX4 quadrotor with a
downward camera, and the autonomy pieces that run on it.

- [1. What this does and how to run it](#1-what-this-does-and-how-to-run-it)
- [2. Status and what is still to be built](#2-status-and-what-is-still-to-be-built) — red-zone avoidance status, **payload drop**
- [3. Repository documentation](#3-repository-documentation)

---

## 1. What this does and how to run it

### What happens in a run

The drone takes off from the launch pad and does this on its own:

1. **Learns the reference.** It hovers over the **start QR** (2 m ahead of the
   pad) and memorises it from the camera.
2. **Sweeps the room.** It flies the corridor, then a lawnmower pattern over
   the 40 × 40 m room at **10 m altitude and 5 m/s** (the competition altitude),
   steering around the red restricted zone as the camera finds it (see
   [Red-zone avoidance](#red-zone-avoidance)).
3. **Finds the match.** Four candidate QR codes lie in the room; only one is
   identical to the start QR. The sweep stops when the QR matcher reports a
   match.
4. **Centres and holds.** It steers onto the matched QR using the QR's position
   in the camera image, then hovers directly above it. (It does not land and
   does not drop anything yet — see part 2.)
5. **Optional:** the camera feed, with the red zone outlined and a magenta cross
   where the drone is, is saved to an MP4 up to this point.
6. **Always:** everything the mission prints, and a position trace, is saved to
   `logs/` (see [Logs](#logs)).

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
# Create a project-specific setup script (adjust the path if your clone is elsewhere)
cat > ~/start_skyscan.sh <<'EOF'
export REPO="$HOME/ML/aerothon_mission2_gazebo_world_v6"
source /opt/ros/jazzy/setup.bash
source "$REPO/QR_Node/ros2_ws/install/setup.bash"
EOF

# 1. Build the ROS workspace (qr_match, red_zone, and their messages)
source /opt/ros/jazzy/setup.bash
export REPO="$HOME/ML/aerothon_mission2_gazebo_world_v6"
cd "$REPO/QR_Node/ros2_ws" && colcon build --packages-select qr_match_interfaces qr_match red_zone

# 2. Install the world and models where Gazebo / PX4 look for them
mkdir -p ~/.simulation-gazebo/models
for m in $REPO/models/*; do rm -rf ~/.simulation-gazebo/models/$(basename $m); cp -r $m ~/.simulation-gazebo/models/; done
cp $REPO/worlds/aerothon_mission2.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/      # PX4 only finds worlds in its own folder
cp $REPO/models/x500_mono_cam_down/model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf
```

Redo step 2 whenever you change anything under `models/` or `worlds/` (for example
the QR tile size, now 1 m), then **fully restart Gazebo** — textures and worlds are
only loaded at start.

If Gazebo is slow on your machine (the simulation running at 0.1–0.3× real time),
the mission still works: its commanded point moves in the simulation's own clock
(see [Simulation speed](#simulation-speed)). It just takes longer in real time.
Running Gazebo headless, or lowering the camera size or rate in
`models/mono_cam/model.sdf`, speeds it up (a smaller image may hurt QR matching).

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
source ~/start_skyscan.sh
source ~/qr_match_venv/bin/activate
ros2 launch qr_match skyscan_qr.launch.py 2>&1 | tee "$REPO/logs/detectors.log"
```
The `tee` also saves the detector output (QR matches, red detections) to `logs/detectors.log`, so it can be read
along with `logs/latest.log` (create the folder first with `mkdir -p "$REPO/logs"`).
Wait for `Ready. Listening on '/camera/image_raw'` from `qr_match_node` (the YOLO
model takes a moment to load).

**Terminal 3 — the mission** (system Python, *not* the venv)
```bash
source ~/start_skyscan.sh
python3 "$REPO/launch/lawnmower_mission.py" --record "$HOME/skyscan_run.mp4"
```
Run `export REPO=~/ML/aerothon_mission2_gazebo_world_v6` first if `$REPO` is empty
in this terminal (otherwise you get `can't open file '/launch/lawnmower_mission.py'`).
`start_skyscan.sh` sets it, so sourcing that is enough.

The video shows a magenta cross where the drone is: the ground point straight
below it (it moves off the image centre when the drone tilts), with altitude and
tilt in the corner. Drop `--record ...` if you don't want the video. Add
`--force-arm` if the drone won't arm. Stop with Ctrl+C (it lands); the video is
only readable after that.

By default the drone **finds the red zone with the camera** and steers around
it; the zone position is not hard-coded any more (see next section). This
needs `red_zone_node` running, which Terminal 2 already starts.

**Terminal 4 (optional) — watch**
```bash
source ~/start_skyscan.sh
ros2 topic echo /qr_match_node/matched
ros2 topic echo /qr_match_node/result --field detections --flow-style   # score + decoded text per QR
ros2 run rqt_image_view rqt_image_view /red_zone_node/debug_image       # red-zone outline on the live feed
```

### Red-zone avoidance

There is no separate node to start. Avoidance runs **inside the mission script**
(Terminal 3), listening to `/red_zone_node/result` from Terminal 2. The code is
in `launch/avoidance.py` (pure Python plus numpy/OpenCV, no ROS, so it can be
tested offline).

**How it works, end to end**
1. **Camera to ground.** Each red patch outline from `red_zone_node` is projected
   onto the floor using the drone's pose *at the time the frame was taken*
   (position, altitude, yaw, **roll and pitch**). Frames are about 0.9 s old when
   they arrive (`--cam-lag`). Frames taken while the drone is tilting more than
   `--max-tilt` are ignored.
2. **The red map.** What was seen is kept as a grid of 0.25 m cells and never
   forgotten. The unseen inside of a zone is filled in (convex hull of what was
   seen), so a zone seen from two sides is treated as solid.
3. **Detours.** Every move is a straight leg along X or Y. If red plus the margin
   is on the leg, the drone goes round it (below) and returns to its row.
4. **Smooth setpoints.** The commanded point speeds up gently, slows for turns and
   stops at detour corners, and is pulled back to within 1 m of the drone the
   moment red is found ahead, so the drone brakes at once.
5. **Coverage.** The camera also records which ground it has looked at. If the
   sweep ends without a match, the drone flies over ground it missed.

```bash
# normal run: red zones are found by the camera, nothing hard-coded
python3 "$REPO/launch/lawnmower_mission.py" --record ~/skyscan_run.mp4

# the old behaviour: route around the hard-coded zone at (32, 8.5), camera not used
python3 "$REPO/launch/lawnmower_mission.py" --static-zone

# tuned: stay 2.5 m from red, camera frames 0.6 s old
python3 "$REPO/launch/lawnmower_mission.py" --margin 2.5 --cam-lag 0.6

# slower, for a bigger safety buffer
python3 "$REPO/launch/lawnmower_mission.py" --speed 1.5

# sweep only the first rows (quick test)
python3 "$REPO/launch/lawnmower_mission.py" --y-max 17.5
```

To test with the zone somewhere else, change the `red_zone_1` pose in
`worlds/aerothon_mission2.sdf`, redo step 2 of the one-time setup, and restart
Gazebo. No change to the script is needed.

The `--record` video is only readable once the mission has exited (the file is
closed on exit), so stop the mission with Ctrl+C before opening it.

**How it goes round a zone and comes back.** Every move is a straight leg along
X or Y: the sweep rows, and the short hops between them. If red (plus the
margin) is on a leg, the drone uses four corners and nothing else:

1. stop just before the red,
2. step sideways until it is clear of the red,
3. run along beside it,
4. drop straight back onto the same row, just past it.

The corners move as more of the red is seen, and the drone keeps its side (left
or right; the smaller detour that stays inside the room). It never cuts across
to the end of the row.

Zones are assumed to be parallel to the walls (the red map keeps the real shape
as a grid of 0.25 m cells, so other shapes work too, but only straight zones have
been stress-tested). The unseen inside of a zone is filled in, so a zone seen from
two sides is treated as solid.

Safety nets:
- A waypoint inside the red slides along its row to the first clear spot (so the
  row is still flown up to the red); if that is not possible it moves to the
  nearest clear spot, or is dropped.
- If the drone ends up clearly inside the margin it backs straight away from the
  red (towards ground the camera has already seen), never towards the unseen side.
- A move that would pass within 0.6 m of red is never flown.
- Where the room wall leaves less than 2 x margin beside a zone, the drone passes
  with a smaller margin (down to 1.0 m) if the row is clear at that margin.
- A waypoint is given up when the drone has not moved 1 m in 40 s, or has no way
  round for 10 s. Nothing is skipped silently: the camera keeps a map of the
  ground it has actually looked at, and when the sweep ends without a match the
  drone flies over any ground it never covered (`revisit` lines), for up to
  `--revisit-time` seconds.
- The commanded point speeds up at `--accel`, slows for turns, stops at detour corners, and is pulled back to within 1 m of the drone the moment red is found ahead, so the drone brakes at once instead of flying on towards where the old plan was heading. The stress test models a drone that lags the setpoint like PX4 (`STRESS_DYNAMIC=0` for an ideal one).
- Once a QR matches, the centring step stops rather than move closer than
  `--centre-clearance` (1.0 m) to red.

**Offline test** (no ROS, no Gazebo; flies the whole mission against simulated
zones, with a delayed camera, and checks the drone never enters a zone, is back on
its row after it, and has all four QRs in view):
```bash
python3 "$REPO/launch/test_avoidance.py"
```
It ends with `19 tests passed`; the table above it shows, per scenario, the
closest approach to the true zone and which QRs were seen.

**Stress test** (also offline): random layouts of 1-3 zones (some touching the
walls), random camera delay, drone speed (random 0.5-1.5 m/s, or fixed with `STRESS_SPEED`), map error and position
noise. A scenario fails if the drone enters red or misses a QR tile that is at
least 2.5 m clear of every zone:
```bash
python3 "$REPO/launch/stress_avoidance.py" 400      # number of scenarios, optional first seed
```
`STRESS_SPEED=5 STRESS_ALT=10 python3 .../stress_avoidance.py 300` fixes the speed and
altitude (also `STRESS_LAG`, `STRESS_MARGIN`, `STRESS_DYNAMIC=0`). Results
(100-1300 scenarios each; random camera delay 0.5-1.3 s unless stated):

| Speed | Altitude | Camera delay | Entered red |
|---|---|---|---|
| 1.5 m/s | 3 m | 0.5-1.3 s | 0 of 200 |
| 3 m/s | 3 m | 0.5-1.3 s | 78 of 200 |
| 5 m/s | 3 m | 0.5-1.3 s | 166 of 200 |
| 5 m/s | 3 m | 0.3 s | 0 of 100 (1 QR missed) |
| 5 m/s | 8 m | 0.9 s | 0 of 100 |
| 5 m/s | 12 m | 0.9 s | 0 of 100 |
| 5 m/s | **10 m** | 0.5-1.3 s | 0 of 600 (closest 0.15 m, median 1.6 m), drone obeying the setpoint instantly |
| 5 m/s | **10 m** | 0.5-1.3 s | 0 of 1300 (closest 0.19 m, median 1.85 m), **drone lagging the setpoint like PX4** (the default now) |

The reason is the camera: at 3 m it sees about 2.7 m ahead of the drone and its
frames are about 1 s old, so at 5 m/s the drone has already flown 4.5 m past what
the map shows. Flying higher (more look-ahead) or a faster camera feed fixes it.
At 5 m/s a QR is also in view for only about a second, so check `--confirm`.

**What you see in Terminal 3 while it runs:**
```
  red map: [x 28.0..36.0, y 4.5..12.5]        the red box found so far (grows as more is seen)
  steering: detour at world (38.3, 7.5)       going round red (also: direct, escape, blocked)
waypoint 8/20 reached: world (14.0, 7.5)
waypoint 11/20 dropped: world (...) is inside the red, or repeats the previous waypoint
waypoint 14/20 given up: no progress for 40 s (drone at world (...))
revisit 1: ground not yet seen near world (x, y); 12 gap tile(s) left
```
Every 3 s it also prints a status line, which is the first thing to check if the
drone does not avoid:
```
  [avoid] loop 20 Hz, sim speed 0.9x real time, drone speed 4.8 m/s (sim) | tilt 6 deg | drone (33.6, 2.5) alt 10.0 yaw 90 | red msgs 222 (with red 27, used 222) | red boxes 1 | qr: 150 results, 94 codes seen, 9 matched | waypoint 6/20 | direct
```
`loop` = how fast the mission loop runs, `sim speed` = how fast the simulation runs
compared with real time (1.0x is ideal; Gazebo on a slow machine can be 0.1-0.2x),
`drone speed` = how fast the drone moves in the simulation's own time (should be
close to `--speed`), `tilt` = the larger of roll and pitch. The commanded point moves
in the simulation's time, not the wall clock, so a slow Gazebo does not make the drone
fly flat out and tilt hard. Camera frames taken at more than `--max-tilt` are ignored,
and smaller tilts are corrected for when the red is placed on the map. `red msgs` = results received from `red_zone_node`; `with red` = how many had red
in them; `used` = how many were matched to a drone pose; `red boxes` = size of the
map. **`yaw` should be about 90** (the drone faces east); if it is 0 the red is
projected sideways. If `red msgs` stays 0, the mission is not receiving
`/red_zone_node/result`. `qr:` counts the QR results, the QR codes seen in them, and how
many of those matched the reference.

`steering:` shows `detour` (going round red), `direct` (nothing in the way),
`escape` (clearly inside the margin, backing out) and `blocked` (no way round
inside the room, holding position).

#### Logs

Everything the mission prints is saved, so nothing needs to be pasted:
- `logs/latest.log` — the terminal output of the newest run (older runs:
  `logs/mission_<date>_<time>.log`).
- `logs/latest_trace.csv` — a position trace every 0.5 s of wall time: time, drone
  x/y, altitude, yaw, commanded point, current target, steering status, waypoint,
  number of red patches.
- `logs/detectors.log` — Terminal 2's output (QR matching, red detections), if you
  use the `tee` in the Terminal 2 command.

`logs/` is git-ignored. To check avoidance from a trace, compute the distance from
each x/y (above 8 m altitude) to the zone rectangle from the world file
(X 28–36, Y 4.5–12.5): a distance of 0 means the drone was over the zone. The trace
is sampled, so a brief clip of a corner between two samples would not show up.

#### Simulation speed

PX4 SITL runs on the simulation's clock, which can run slower than real time when
Gazebo is heavily loaded. The mission therefore moves its commanded point in the
**simulation's** time (from PX4's `time_boot_ms`), not the wall clock; moving it in
wall time made the point run far ahead of a slow drone, so PX4 flew flat out and
tilted hard (the camera then saw the horizon and the red map smeared). The
`[avoid]` line shows `sim speed` (1.0x is real time) and `drone speed` (should be
near `--speed`).

**Options**

| Flag | Default | Meaning |
|---|---|---|
| `--log-dir` | `logs/` | where the log and trace are saved (`''` = off) |
| `--margin` | `2.0` | keep this far (m) from any red seen. At 10 m altitude the camera sees about 9 m ahead, so a zone met head-on is noticed in good time; at 3 m it sees only about 2.7 m. A margin of 3.0 m did not improve the worst case in the stress test |
| `--cam-lag` | `0.9` | age (s) of a camera frame when its detection arrives, used to match the detection with the drone's pose. If the `red map:` box is shifted along the flight direction, change this: too small shifts it ahead of the drone, too large shifts it behind |
| `--revisit-time` | `300` | after the sweep, up to this long (s) flying over ground the camera never covered |
| `--alt` | `10` | cruise altitude (m); the competition altitude |
| `--speed` | `5.0` | route speed (m/s). **At 3 m altitude this is too fast to avoid red**; at 10 m it is fine in the offline tests (see below) |
| `--accel` | `3.0` | the commanded point speeds up / slows down at this rate (m/s^2); lower = gentler, less tilt |
| `--max-tilt` | `0.12` | ignore camera frames taken while roll or pitch is more than this (rad, 0.12 = 7°); a tilted camera places the red wrongly if the pose and picture are out of step |
| `--ref-alt` | `2.0` | height (m) over the start QR when the reference is captured |
| `--y-max` | `37.5` | sweep only rows up to this y (m) |
| `--centre-clearance` | `1.0` | while centring on a matched QR, never move closer than this (m) to red |
| `--confirm` | `2` | consecutive `matched` messages needed to stop the sweep |
| `--max-lead` | `1.1 x speed` | the commanded point never runs more than this far (m) ahead of the drone |
| `--static-zone` | off | use the hard-coded zone instead of the camera |

**Watch it work** (extra terminal):
```bash
source /opt/ros/jazzy/setup.bash
source "$REPO/QR_Node/ros2_ws/install/setup.bash"
ros2 topic echo /red_zone_node/detected                                   # red in the frame right now
ros2 run rqt_image_view rqt_image_view /red_zone_node/debug_image         # the red outline the drone is using
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
Order of things the drone passes: `qr_c`, `qr_d`, **the red zone (it steers
around it)**, then `qr_b` (the correct one). `qr_c` and `qr_d` must **not** match.
`Ctrl+C` lands the drone.

Run `python3 lawnmower_mission.py --help` for every option. The main ones: `--alt`
(10 m), `--speed` (5 m/s), `--record FILE.mp4`, `--record-topic` (default is the
red-zone-annotated feed; use `/camera/image_raw` for the raw one), `--force-arm`,
the centring tuning `--center-tol / --center-gain / --max-step / --settle`, and the
avoidance options in the table above.

---

## 2. Status and what is still to be built

### A. Red-zone avoidance — built

Details, commands and options are in [Red-zone avoidance](#red-zone-avoidance) in
part 1. The hard-coded zone is gone (`--static-zone` keeps the old route for
comparison): the drone finds the zone with the camera and goes round it.

**Checked in Gazebo** (10 m, 5 m/s, zone at (32, 8.5)): the drone never entered the
zone; it passed 2 m from its edge on row 2.5, and met it head-on on row 7.5 by
stopping 2.6 m short, going round the north side and returning to the row. That
run was stopped part-way through the sweep (waypoint 11 of 20).

**Checked offline only** (`test_avoidance.py`, `stress_avoidance.py`): 1300 random
layouts of 1–3 zones at 5 m/s and 10 m with a PX4-like lagging drone, 0 entries
into red (closest approach 0.19 m, median 1.85 m). The offline drone is simpler than
a real one, and the camera delay is a model.

**Not yet verified, or known limits**
- A whole sweep at 5 m/s in Gazebo, and zones in other places than (32, 8.5).
- Zones turned at an angle to the walls. The red map keeps the real shape, so it
  should work, but only zones parallel to the walls have been stress-tested.
- **QR matching at 10 m.** The reference is captured at 2 m (`--ref-alt`) but the
  search is at 10 m, where a 1 m tile is about 54 px wide. If matching is weak, try
  `--ref-alt 10`. This has not been tested.
- At 5 m/s a QR is in view for only a few seconds; `--confirm 2` may need
  attention.
- A zone only 0.7 m from a wall, on a hop along that wall, is the one stress case
  that still failed earlier; a zone covering the corridor mouth cannot be passed at
  all.
- At 3 m altitude, 5 m/s is **not** safe: the camera sees only about 2.7 m ahead
  and its frames are about 1 s old (see the stress table).
- The red map can be larger than the zone (it was about 2–3 m too big on two sides
  in one run) because of tilt at corners. This only makes the drone more cautious.
- Geo-fence coordinates are "to be provided in Phase 2" in the rulebook; nothing
  in the repo encodes a fence.

The HSV thresholds are in `red_zone/config/red_zone.yaml`. The QR tile borders are
deliberately **not** red (blue / teal / purple) so the detector does not fire on
them — keep it that way if you add tiles.

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
   within a set distance of the QR centre (≈ 1 m tile). Report the result.
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
│   ├── lawnmower_mission.py       the mission: takeoff, read reference, sweep, centre, hold, (record), logs
│   ├── avoidance.py               red-zone avoidance (projection, red map, detours, setpoints, coverage)
│   ├── test_avoidance.py          offline tests (19)
│   └── stress_avoidance.py        offline random stress test
├── logs/                          mission logs and traces (created on the first run, git-ignored)
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
| Entrance banners | X = 3 and X = 13, panels at Z 4.2–5.8 m (the mission cruises at 10 m, above them) |
| Room | X 13–53, Y 0–40, 2.5 m walls, 7 m gap in the west wall at Y 16.5–23.5 |
| **Red zone** | **centre `(32, 8.5)`, 8 × 8 m** (X 28–36, Y 4.5–12.5; the sweep row at Y = 7.5 runs through it, between `qr_d` and `qr_b`), flat, no collision (a no-fly zone for the autonomy, not a wall) |
| Candidate QRs (1 m tiles) | `qr_a (20, 32)`, `qr_b (18, 8)`, `qr_c (24, 4)`, `qr_d (46, 7.5)` |

The **correct** QR is `qr_b`: its image is identical to `qr_start`. Sweep order
puts it third among the QRs, with the red zone crossed before it
(`qr_c`, `qr_d`, red zone, `qr_b`), so a matcher that wrongly matches the first
QR it sees fails the test.

### Models (`models/`)

| Model | What |
|---|---|
| `qr_start`, `qr_a`–`qr_d` | flat 1 m QR tiles. `qr_e` exists but is not placed in the world |
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
topics. Avoidance lives in `launch/avoidance.py`.

1. Starts logging to `logs/` (terminal output and a trace).
2. Streams setpoints, switches to offboard, arms, climbs to `--alt` (10 m).
3. Flies to the start QR at `--ref-alt` (2 m), calls `capture_reference`, waits
   for `reference_ready`. The start QR position `(2, 20)` is hard-coded from the
   rulebook layout; the camera is not used to find it.
4. `build_route()`: corridor (forward lane) → serpentine over the room, 8 rows 5 m
   apart (`ROW_SPACING`). The rows are plain; zones are handled while flying.
5. Flies the route: `Navigator` hands out legs, `Detour` plans round red, `Slew`
   moves the commanded point (in PX4's clock, at `--speed`, accelerating at
   `--accel`). Stops after `--confirm` (2) consecutive `matched = true` — **ignored
   while X < 13.5 m**, because the start QR matches itself.
6. Centring loop: takes the matched QR's pixel box, converts the offset from
   image centre to metres, moves the hold point by `--center-gain` (0.4) of it,
   capped at `--max-step`, never closer than `--centre-clearance` to red, waits
   for the drone to arrive and settle, skips the results still in flight,
   repeats until 3 readings are within `--center-tol`.
7. Stops the recording, then hovers until Ctrl+C (then lands).

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
- **Paste commands one per line.** Two commands run together on one line are passed
  to the first as arguments (e.g. `rqt_image_view ... source ~/start_skyscan.sh`).
- **A slow Gazebo** shows up as `sim speed` well below 1.0x in the `[avoid]` line.
  Everything still works (see [Simulation speed](#simulation-speed)), only slower.
- **Terminals running more than one ROS setup:** the offline fake-world tests used
  `ROS_DOMAIN_ID` 80; your normal terminals should not set it.
- `launch/__pycache__/` holds a tracked `.pyc` file; run Python with
  `PYTHONDONTWRITEBYTECODE=1` to keep it from showing up as modified.

