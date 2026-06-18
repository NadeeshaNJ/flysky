# FlySky — QBot

An interactive **companion robot** that recognises its owner and responds to hand
gestures. Vision from a **Kinect 360** RGB-D camera; motion on a **Kobuki / Quanser
QBot** base; software in **ROS 2 Jazzy** on a **Raspberry Pi 5**.

> Team **FlySky** · See [`team_flysky_project.md`](team_flysky_project.md) for the
> proposal & timeline and [`QBOT_OVERVIEW.md`](QBOT_OVERVIEW.md) for the system design.

## What it does
Detects and tracks a person, recognises a small set of hand gestures, and reacts
with pet-like behaviours — approach, stop, spin, step left/right, and wiggle —
with audio feedback and depth-aware safety.

## Architecture
This repository **is** the ROS 2 (`colcon`) workspace.

| Package | Node | Publishes |
|---|---|---|
| `kinect_camera` | `kinect_rgbd` | `/kinect/rgb/image_raw`, `/kinect/depth/image_raw` |
| `vision_node` | `face_tracker_node` | `/vision/target` |
| `gesture_node` | `gesture_command_node` | `/gesture/tracking` |
| `behavior_node` | `pet_behavior_node` | `/cmd_vel`, `/commands/sound` |

## Quick start
> Full step-by-step setup, hardware notes, and troubleshooting are in
> [`INSTALL.md`](INSTALL.md).

```bash
# 1. Install ROS 2 Jazzy + all dependencies + Kinect USB permissions
#    (needs sudo — you'll be prompted for your password)
bash setup_qbot_env.sh

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Run the whole system
ros2 launch behavior_node qbot.launch.py
```

## Status
Project scaffold. All nodes build and run end-to-end as working stubs; the Kinect
driver, gesture classifier, and Kobuki base driver are completed as the hardware
is connected. See [`CLAUDE.md`](CLAUDE.md) for layout, conventions, and the
remaining hardware-dependent tasks.

## References
- Kobuki / QBot: <https://kobuki.readthedocs.io/en/devel/>
- OpenCV: <https://docs.opencv.org/4.x/>
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- libfreenect (Kinect): <https://github.com/OpenKinect/libfreenect>
