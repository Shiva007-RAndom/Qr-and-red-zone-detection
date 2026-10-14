# PX4 + Gazebo Downward Camera: Troubleshooting and Fix

## Goal

Run PX4 SITL with the `x500_mono_cam_down` model in a custom Gazebo
world containing a green lawn and a red patch, then receive the downward
camera image through ROS 2 for OpenCV-based red-zone detection.

## Environment

-   Ubuntu
-   ROS 2 Jazzy
-   Gazebo Sim 8 / Harmonic
-   PX4 Autopilot SITL
-   `ros-jazzy-ros-gz-bridge`

## Symptoms

The Gazebo GUI displayed the green lawn and red patch, and the ROS 2
camera topic was publishing at roughly 25--28 Hz. Initially, image
inspection reported:

``` text
Dimensions: 1280 x 960
Encoding: rgb8
Step: 3840
Pixel range: 218 to 218
Unique pixel values: 1
```

The image messages were arriving, but all pixels had the same value. An
active topic and a healthy message rate alone did not prove that the
camera was rendering the scene correctly.

## Camera topic

``` text
/world/minimal_drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

The Gazebo topic was confirmed to have a publisher of type
`gz.msgs.Image`, and ROS 2 measured a rate of approximately 25--28 Hz.

## Fix: Avoid specifying the same camera rotation in two places

File:

``` text
~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf
```

The model initially specified a 90-degree pitch both on the included
camera model and on the fixed joint. The configuration was changed so
that the included model has no rotation, while the fixed joint carries
the pitch rotation.

Use:

``` xml
<include merge='true'>
  <uri>x500</uri>
</include>

<include merge='true'>
  <uri>model://mono_cam</uri>
  <pose>0 0 .10 0 0 0</pose>
  <name>mono_cam</name>
</include>

<joint name="CameraJoint" type="fixed">
  <parent>base_link</parent>
  <child>camera_link</child>
  <pose relative_to="base_link">0 0 0 0 1.5707 0</pose>
</joint>
```

Keep the sensor pose in
`~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` as:

``` xml
<sensor name="camera" type="camera">
  <gz_frame_id>camera_link</gz_frame_id>
  <pose>0 0 0 0 0 0</pose>
  <camera>
    <horizontal_fov>1.74</horizontal_fov>
    <image>
      <width>1280</width>
      <height>960</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>3000</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

The important change was moving the 90-degree pitch from the included
model pose to the fixed joint pose, rather than specifying it in both
locations.

## Gazebo environment variables

In the simulation terminal:

``` bash
source /opt/ros/jazzy/setup.bash

export GZ_CONFIG_PATH="/usr/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz"

export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/worlds:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/.simulation-gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

Launch PX4 SITL and Gazebo together:

``` bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=minimal_drone_world make px4_sitl gz_x500_mono_cam_down
```

After changing an SDF file, stop PX4 and Gazebo completely and relaunch
so the model is reloaded.

## ROS 2 bridge

In a separate terminal:

``` bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  '/world/minimal_drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
```

Check the rate in another terminal:

``` bash
source /opt/ros/jazzy/setup.bash

ros2 topic hz \
  /world/minimal_drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

A rate around 25--28 Hz was observed with the sensor configured for 30
Hz. Exact rates vary with machine load and simulation performance.

## Verify image contents

After the pose adjustment, the image check reported:

``` text
Dimensions: 1280 x 960
Encoding: rgb8
Step: 3840
Pixel range: 1 to 218
Unique pixel values: 191
```

This is an improvement over the original constant image because the
pixel values are no longer uniform. It proves image variation, but does
not alone prove that the lawn and red patch are visible. Inspect a saved
frame to confirm scene content.

## Save a camera frame

Create `~/PX4-Autopilot/save_frame.py`:

``` python
import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TOPIC = (
    "/world/minimal_drone_world/model/x500_mono_cam_down_0/"
    "link/camera_link/sensor/camera/image"
)

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_frame_saver")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, TOPIC, self.callback, 10
        )
        self.saved = False

    def callback(self, msg):
        if self.saved:
            return

        image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )

        cv2.imwrite("camera_frame.png", image)

        print("Dimensions:", image.shape[1], "x", image.shape[0])
        print("Pixel range:", image.min(), "to", image.max())
        print("Unique pixel values:", len(np.unique(image)))
        print("Saved camera_frame.png")

        self.saved = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Install dependencies if needed:

``` bash
sudo apt install ros-jazzy-cv-bridge python3-opencv
```

Run while PX4, Gazebo, and the ROS-Gazebo bridge are active:

``` bash
source /opt/ros/jazzy/setup.bash
cd ~/PX4-Autopilot
python3 save_frame.py
```

Open the image:

``` bash
xdg-open ~/PX4-Autopilot/camera_frame.png
```

## Troubleshooting commands

List relevant Gazebo topics:

``` bash
gz topic -l | grep -E 'imu|baro|camera/image'
```

Inspect camera publisher/subscriber information:

``` bash
gz topic -i -t \
  /world/minimal_drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

Read camera messages directly from Gazebo:

``` bash
gz topic -e -t \
  /world/minimal_drone_world/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

The image messages are large, so the last command may be verbose. Stop
it with `Ctrl+C`.

## Separate issue: PX4 preflight warnings

During testing, PX4 also reported:

``` text
Preflight Fail: No valid data from Accel 0
Preflight Fail: barometer 0 missing
Preflight Fail: No valid data from Gyro 0
Preflight Fail: No connection to the GCS
```

These warnings were not resolved by the camera pose change and should be
investigated separately. The IMU topic found in the topic list was:

``` text
/world/minimal_drone_world/model/x500_mono_cam_down_0/link/base_link/sensor/imu_sensor/imu
```

A prior attempt to echo a topic under `link/imu_link/` used a topic name
that was not present in the listed topics. Do not assume the
IMU/preflight issue is fixed just because camera image messages arrive.

## Current status

-   Gazebo GUI shows the green lawn and red patch.
-   The camera topic has an active Gazebo image publisher.
-   ROS 2 image messages arrive at roughly 25--28 Hz.
-   After the pose adjustment, pixel data varies (`1–218`, 191 unique
    values), instead of being a single constant value.
-   **Still to verify:** visually confirm that `camera_frame.png`
    contains the ground and red patch.
-   **Separate and unresolved:** PX4 accelerometer, gyro, barometer, and
    GCS preflight warnings.
