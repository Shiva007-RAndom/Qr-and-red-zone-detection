# Running and deploying qr_match

This covers the parts not in `README.md` (which describes the algorithm and
parameters): how to actually set up, build, and run this on a dev machine,
and how to carry it over to the drone's full codebase. Steps below were
verified on Ubuntu 24.04 / ROS 2 Jazzy, CPU only (no GPU).

## 1. Local setup

### 1a. System packages

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions libzbar0 \
    ros-jazzy-cv-bridge ros-jazzy-vision-opencv
```

Only needed if you plan to test with static images instead of a real camera
(see [Testing without a camera](#testing-without-a-camera)):

```bash
sudo apt install ros-jazzy-image-publisher
```

### 1b. Python dependencies — in a venv

`qrdet` pulls in `torch`/`ultralytics`, which drags in a `numpy` version
newer than the one Debian's `python3-opencv` (needed by `cv_bridge`) was
built against. Installing it system-wide risks breaking `cv_bridge`'s numpy
ABI compatibility, so it goes in a venv instead:

```bash
python3 -m venv ~/qr_match_venv --system-site-packages
source ~/qr_match_venv/bin/activate

# CPU-only torch (skip if the machine has an NVIDIA GPU you want to use —
# drop --index-url in that case to get the full CUDA build)
pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
pip install qrdet pyzbar
```

`--system-site-packages` lets the venv still see `rclpy`/`cv_bridge`/message
packages, which are apt packages, not pip ones.

**Important gotcha:** `colcon` itself is an apt package with a hardcoded
system-Python shebang, so building inside an active venv does *not* make the
installed `qr_match_node` script use the venv's interpreter — it always runs
under system Python regardless. Since `qrdet`/`torch`/`pyzbar` are pure
Python (nothing needs compiling against the venv's interpreter specifically),
the fix is to put the venv's site-packages on `PYTHONPATH` at *run* time
instead of trying to change the interpreter. Append this to
`~/qr_match_venv/bin/activate` once, so a plain `source
~/qr_match_venv/bin/activate` always sets it up correctly:

```bash
cat >> ~/qr_match_venv/bin/activate << 'EOF'

# make venv packages importable by system-python ROS entry-point scripts
export PYTHONPATH="$HOME/qr_match_venv/lib/python3.12/site-packages:$PYTHONPATH"
EOF
```

(Adjust `python3.12` to match your system's Python version if different.)

### 1c. Build

```bash
source /opt/ros/jazzy/setup.bash   # swap in your distro
cd ~/ML/QR_Node/ros2_ws
colcon build --packages-select qr_match_interfaces qr_match
```

Build `qr_match_interfaces` first (or together, as above) since `qr_match`
imports its messages. The venv does not need to be active for this step —
see the gotcha above.

### 1d. Reference and test images

Drop a reference QR (and any test QR images) into
`src/qr_match/test_data/`. This folder is plain source data — it is never
built, installed, or read by `setup.py`, so it can hold anything without
touching the ROS build.

---

## 2. Running

### Against a real camera

```bash
source /opt/ros/jazzy/setup.bash
source ~/qr_match_venv/bin/activate   # needed for qrdet/torch on PYTHONPATH
cd ~/ML/QR_Node/ros2_ws && source install/setup.bash

ros2 launch qr_match qr_match.launch.py \
    reference_qr_path:=/absolute/path/to/reference_qr.png \
    image_topic:=/camera/color/image_raw
```

Or directly, without the launch file:

```bash
ros2 run qr_match qr_match_node --ros-args \
    -p reference_qr_path:=/absolute/path/to/reference_qr.png \
    -p image_topic:=/camera/image_raw \
    -p match_threshold:=0.85
```

This process is meant to run for as long as the camera is streaming — for a
drone, that means the whole flight. It does not exit on its own; that is
correct, not a bug.

Watch the output live:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ML/QR_Node/ros2_ws/install/setup.bash   # required for the custom QRResult type
ros2 topic echo /qr_match_node/matched
ros2 topic echo /qr_match_node/result
```

### Testing without a camera

`qr_match_test_feed` is a small one-shot node that stands in for a camera: it
publishes every image in a folder, one at a time, waits for
`qr_match_node`'s verdict on that exact frame, prints it, and exits once the
folder is exhausted. `qr_match_node` itself is untouched and keeps running
throughout, exactly as it will with a real camera.

Terminal 1 — the node under test, run exactly as above:
```bash
source /opt/ros/jazzy/setup.bash
source ~/qr_match_venv/bin/activate
cd ~/ML/QR_Node/ros2_ws && source install/setup.bash
ros2 launch qr_match qr_match.launch.py \
    reference_qr_path:=/absolute/path/to/reference_qr.png \
    image_topic:=/image_raw
```

Terminal 2 — feed it the test folder and exit when done:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ML/QR_Node/ros2_ws/install/setup.bash
ros2 run qr_match qr_match_test_feed ~/ML/QR_Node/ros2_ws/src/qr_match/test_data
```

Example output:
```
dummy_qr1.png    MATCH     best=1.000 detections=1 (313 ms)
dummy_qr2.png    no match  best=0.000 detections=1 (314 ms)
dummy_qr3.png    no match  best=0.000 detections=1 (342 ms)
dummy_qr4.png    no match  best=0.517 detections=1 (358 ms)
```

`qr_match_test_feed` never needs `qrdet`/`torch`, so it runs fine from system
Python — the venv only needs to be active in the terminal running
`qr_match_node`. By default it excludes `reference_qr.png` from the folder
(`--exclude` to change) and skips to the next image after a 5 s timeout if no
result arrives (`--timeout`).

---

## 3. Deploying onto the drone's full codebase

The two packages here (`qr_match`, `qr_match_interfaces`) are self-contained
and don't touch anything outside themselves — dropping them into the drone's
main workspace is a copy, not a merge.

1. **Copy both package folders** into the drone's workspace:
   ```bash
   cp -r qr_match_interfaces qr_match /path/to/drone_ws/src/
   ```

2. **Repeat the local setup above on the drone's onboard computer**: system
   packages (step 1a), the venv with the `PYTHONPATH` patch (step 1b) — this
   is required on every machine that runs `qr_match_node`, not just this dev
   box.

3. **Build the whole drone workspace** as usual; `qr_match`/
   `qr_match_interfaces` build the same way whether alone or alongside other
   packages:
   ```bash
   cd /path/to/drone_ws
   colcon build
   ```

4. **Point `image_topic` at the drone's real camera driver topic** and set
   `reference_qr_path` to wherever the reference QR image is deployed on the
   drone's filesystem — either pass them as launch arguments from the
   drone's master launch file, or edit `config/qr_match.yaml` and load it via
   `ros2 param load` / a `parameters=[...]` entry in the master launch file.
   Check the real camera driver's QoS: if it publishes `RELIABLE` instead of
   `BEST_EFFORT`, set `best_effort_qos:=false` or the subscription won't
   connect.

5. **Include `qr_match.launch.py` from the drone's top-level launch file**
   (or add an equivalent `Node(...)` entry directly) so it starts
   automatically alongside the rest of the flight stack, rather than being
   launched by hand. `qr_match_node` has no dependency on any other node in
   this repo, so it can be added to the flight launch file independently of
   startup order — it just needs the camera topic to be publishing by the
   time frames should start being processed (it will happily sit idle until
   then).

6. **Leave `qr_match_test_feed` and `test_data/` in the repo** even after a
   real camera is wired up — they cost nothing (they aren't invoked
   automatically), and stay useful for regression-testing threshold changes
   offline before a flight.

7. **First boot needs internet**: `qrdet` downloads its YOLO weights on
   first use and caches them afterwards (under `~/.config/Ultralytics/` and
   the `qrdet` package's model cache). If the drone's onboard computer won't
   have internet on its first real run, pre-warm the cache by running the
   node once (e.g. through `qr_match_test_feed`) while it's still connected,
   then copy that cache directory over if deploying to a different machine
   image.
