# qr_match

A ROS 2 node that detects QR codes in an image stream and reports whether any
of them match a reference QR code given at startup. Ported from the
`QR_detection.ipynb` notebook.

## What it does

For every incoming image:

1. **Detect** — `qrdet` (a YOLO model) finds every QR code in the frame.
2. **Warp** — each QR is perspective-corrected into a 300×300 square, so
   angled or tilted views are flattened.
3. **Normalise** — adaptive thresholding removes uneven lighting.
4. **Sample** — the square is sampled into an *N*×*N* grid of modules.
5. **Score** — the grid is compared against the reference grid over all four
   rotations; the score is the fraction of modules that agree, in `[0, 1]`.
6. **Publish** — `matched = score >= match_threshold` (default **0.85**).

If both the reference and a candidate decode cleanly, their payloads are
compared directly instead. Set `trust_decoded_text: false` to always use the
grid score.

## Topics

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| sub | `image_topic` (param) | `sensor_msgs/Image` or `CompressedImage` | input frames |
| pub | `~/detected` | `std_msgs/Bool` | was any QR found |
| pub | `~/matched` | `std_msgs/Bool` | did any QR match the reference |
| pub | `~/result` | `qr_match_interfaces/QRResult` | full per-QR detail |
| pub | `~/reference_ready` | `std_msgs/Bool` | a reference QR is loaded |
| srv | `~/capture_reference` | `std_srvs/Trigger` | use the best QR in the next frame as the reference |

The three published topics are latched (`TRANSIENT_LOCAL`), so a subscriber
that starts late immediately receives the most recent verdict.

`QRResult` carries the source image header, `qr_detected`, `num_detections`,
`match_found`, `best_score`, the applied threshold, `processing_time_ms`, and a
`detections[]` array sorted by descending score (each with bounding box,
detector confidence, score, matched flag, and decoded text).

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `reference_qr_path` | `""` | Optional. Absolute path to a reference QR image. If empty, call the `~/capture_reference` service to learn the reference from the camera; until then nothing matches. |
| `image_topic` | `/camera/image_raw` | Input topic. |
| `use_compressed` | `false` | Subscribe to `CompressedImage` instead. |
| `best_effort_qos` | `true` | Most camera drivers publish best-effort. |
| `match_threshold` | `0.85` | Score at or above which a QR counts as a match. |
| `trust_decoded_text` | `true` | Prefer exact payload comparison when both decode. |
| `model_size` | `s` | qrdet YOLO size: `n`, `s`, `m`, `l`. |
| `conf_threshold` | `0.5` | Detector confidence floor. |
| `process_every_n` | `1` | Raise to skip frames on slower hardware. |
| `log_every_frame` | `false` | `false` logs only frames containing a match. |

## Install

```bash
# ROS 2 (Humble shown; swap in your distro)
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv

# Python
pip3 install qrdet

# Optional, for more reliable payload decoding
sudo apt install libzbar0 && pip3 install pyzbar
```

`qrdet` pulls in `ultralytics`/`torch` and downloads its YOLO weights on first
run, so the machine needs internet the first time the node starts. Weights are
cached afterwards.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select qr_match_interfaces qr_match
source install/setup.bash
```

Build `qr_match_interfaces` first (or together, as above) since `qr_match`
imports its messages.

## Run

```bash
ros2 launch qr_match qr_match.launch.py \
    reference_qr_path:=/absolute/path/to/dummy_qr.png \
    image_topic:=/camera/color/image_raw
```

Or directly:

```bash
ros2 run qr_match qr_match_node --ros-args \
    -p reference_qr_path:=/absolute/path/to/dummy_qr.png \
    -p image_topic:=/camera/image_raw \
    -p match_threshold:=0.85
```

Watch the output:

```bash
ros2 topic echo /qr_match_node/matched
ros2 topic echo /qr_match_node/result
```

## Choosing the threshold

Measured over 40 randomly generated 20-character QR payloads compared against
one reference:

| Case | Score |
|---|---|
| Same payload, re-rendered at four different scales | 1.000 |
| Unrelated payloads (n=40) | 0.612 – 0.753, mean 0.667 |

Unrelated QRs never score near zero because every QR shares the same finder,
timing and alignment patterns — about two thirds of modules agree by
construction. So the useful decision boundary is roughly 0.78–0.90, and the
0.85 default sits comfortably in the gap. Raise it if you see false positives;
lower it if genuine matches are missed under bad lighting. Validate against
footage from your actual camera before trusting a tighter threshold — blur and
glare push real matches down.

## Layout

```
qr_match_interfaces/     custom messages (QRResult, QRDetection)
qr_match/
  qr_match/qr_match_node.py   the whole node: pipeline + ROS wrapper
  launch/qr_match.launch.py
  config/qr_match.yaml
```

Everything lives in one Python file, `qr_match_node.py`: the pipeline is a
handful of plain functions at the top (no ROS, no shared state), and the
`QRMatchNode` class wraps them.

## Notes on the port

A few things were changed from the notebook, all for correctness:

* **Corner ordering.** `cv2.getPerspectiveTransform` needs corners in a known
  order. The notebook passed qrdet's points unsorted, which can silently
  produce mirrored or twisted warps. Corners are now sorted clockwise from the
  top-left.
* **Module count.** The notebook picked whichever QR version divided the 300 px
  warp most evenly, which always answered 25. It's now estimated from the
  median run length along scan lines, and candidates are sampled at the
  reference's module count so the grids are directly comparable.
* **Patch sampling.** Each module is read as the median of a small patch rather
  than a single pixel, which is far less sensitive to sensor noise.
* **Reference fallback.** If the detector finds no QR in the reference image
  (common with a cleanly cropped PNG), the whole image is used after trimming
  the quiet zone, instead of the notebook's hard assertion failure.

## Performance

The YOLO detector dominates the per-frame cost and runs inside the callback.
On CPU expect a few FPS at `model_size: s`; use `model_size: n`, raise
`process_every_n`, or provide a GPU to keep up with a fast camera. The
subscriber queue depth is 1, so under load the node processes the most recent
frame rather than a backlog.
