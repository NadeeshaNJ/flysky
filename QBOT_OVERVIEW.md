# QBot - Autonomous Vision-Based Mobile Robot

## Physical Tasks

QBot is a mobile robot that detects and responds to human visual signals:

- **Face Detection & Tracking** — Identifies and tracks human faces from camera input
- **Gesture Recognition** — Recognizes hand gestures as commands (beckon, palm wave, index finger circles, left/right waves)
- **Motion Control** — Executes robot behaviors:
  - Move forward (approach/come closer)
  - Stop/hold position
  - Rotate in place
  - Move left or right (one foot distance)
  - Oscillate left-right (tail wag)
  - Re-center orientation toward detected hand or face

The robot operates as a mobile pet-like platform with depth-aware collision avoidance and audio feedback for each command.

## ROS 2 Nodes (Jazzy)

| Node | Package | Role |
|------|---------|------|
| **kinect_rgbd** | kinect_camera | Publishes RGB and depth images from Kinect v1/360 sensor |
| **face_tracker_node** | vision_node | Detects faces from RGB feed; publishes face position/size |
| **gesture_command_node** | gesture_node | Analyzes hand landmarks and classifies gesture commands |
| **pet_behavior_node** | behavior_node | Subscribes to vision and gesture topics; publishes velocity commands and sound cues |

## Key Topics

- Input: `/kinect/rgb/image_raw`, `/kinect/depth/image_raw`
- Output: `/cmd_vel` (velocity), `/commands/sound` (audio feedback)
- Internal: `/vision/target`, `/gesture/tracking`

## Kinect 360 Camera Setup (Raspberry Pi)

The RPi 5 connects to an Xbox Kinect v1 / Kinect for Windows device over USB.

### Connection & Hardware Requirements

- **USB Connection**: Plug the Kinect directly into a USB 3.0 port on the RPi 5 (or compatible USB 2.0 with reduced bandwidth)
- **Device IDs**: Linux identifies the Kinect via USB vendor `045e` (Microsoft) with multiple product IDs:
  - Motor: `02c2`, `02b0`
  - Audio: `02ad`
  - Camera: `02ae`

### RPi Setup Steps

1. **Verify Kinect Detection**
   ```bash
   lsusb | grep 045e
   ```
   Should show Motor, Audio, and Camera devices.

2. **Grant User Permissions** (if not already done by `setup_qbot_env.sh`)
   - Ensure the `flysky` user is in `video` and `plugdev` groups
   - Install udev rules for OpenKinect to allow unprivileged USB access

3. **Build libfreenect Locally** on the RPi
   ```bash
   cd /tmp
   git clone --depth 1 https://github.com/OpenKinect/libfreenect
   cmake -S /tmp/libfreenect -B /tmp/libfreenect/build \
     -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON=OFF -DBUILD_CPP=OFF -DBUILD_CV=OFF
   cmake --build /tmp/libfreenect/build -j$(nproc)
   ```

4. **Download Kinect Firmware** (required for Kinect for Windows)
   ```bash
   mkdir -p ~/.libfreenect
   cd /tmp/libfreenect/src
   python3 ./fwfetcher.py ~/.libfreenect/audios.bin
   ```

5. **Build the ROS 2 Kinect Package**
   ```bash
   cd ~/qbot_ws
   LIBFREENECT_ROOT=/tmp/libfreenect colcon build --packages-select kinect_camera \
     --cmake-args -DLIBFREENECT_ROOT=/tmp/libfreenect
   source install/setup.bash
   ```

6. **Run the Kinect Node** (with environment setup)
   ```bash
   source ~/qbot_ws/install/setup.bash
   export LD_LIBRARY_PATH=/tmp/libfreenect/build/lib:$LD_LIBRARY_PATH
   export LIBFREENECT_FIRMWARE_PATH=$HOME/.libfreenect
   ros2 launch kinect_camera kinect_rgbd.launch.py
   ```

The `kinect_camera` package uses the low-level libfreenect C API to open only the camera subdevice (avoiding motor/LED errors on Windows hardware) and publishes RGB and depth frames at 30 Hz by default.

### Known Issues & Fixes

- **LED/Motor errors** (`LIBUSB_ERROR_IO`): Harmless on Windows hardware; streams still publish if firmware is present.
- **No device found errors**: Ensure libfreenect build is complete and firmware blob exists before starting the node.
- **Permissions denied**: Log out and back in after adding groups, or use `sudo` as a quick test.
