# CLAUDE.md — QBot / FlySky project

Guidance for Claude Code (and humans) working in this repo.

## What this is
**FlySky** is building **QBot**: a pet-like companion robot that recognises its
owner and reacts to hand gestures. Vision comes from a **Kinect 360 (Xbox Kinect
v1)** RGB-D camera; the mobile base is a **Kobuki / Quanser QBot**. Everything
runs on **ROS 2 Jazzy**.

Read `QBOT_OVERVIEW.md` (technical/system overview) and
`team_flysky_project.md` (project proposal, timeline, task split) for full context.

## Hardware / environment
- **Board:** Raspberry Pi 5, 4 GB RAM — keep things light (ros-base, not desktop;
  Python nodes over heavy C++ where reasonable).
- **OS:** Ubuntu 24.04 LTS (Noble), arm64.
- **ROS distro:** Jazzy.
- **Camera:** Kinect v1 / 360 over USB (libfreenect). Vendor `045e`.
- **Base:** Kobuki (kobuki.readthedocs.io). *Not yet connected — wire-up pending.*

## Repository layout
This repo **is** the colcon workspace (referred to as `qbot_ws` in the docs).

```
src/
  kinect_camera/   kinect_rgbd            -> /kinect/rgb/image_raw, /kinect/depth/image_raw
  vision_node/     face_tracker_node      -> /vision/target
  gesture_node/    gesture_command_node   -> /gesture/tracking
  behavior_node/   pet_behavior_node      -> /cmd_vel, /commands/sound
                   launch/qbot.launch.py  (full-system bringup)
setup_qbot_env.sh  one-shot environment installer (ROS 2 + deps + Kinect udev)
```

All four packages are `ament_python`.

## Common commands
```bash
# One-time environment install (ROS 2 Jazzy + deps + Kinect permissions)
bash setup_qbot_env.sh

# Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Run a single node / the whole system
ros2 launch kinect_camera kinect_rgbd.launch.py
ros2 launch behavior_node qbot.launch.py

# Inspect
ros2 topic list
ros2 topic echo /vision/target
ros2 topic echo /cmd_vel
```

## Status & conventions
- Node bodies are **working scaffolds**: the graph wires up and runs end-to-end,
  but the Kinect driver, gesture classifier, and Kobuki base driver are filled in
  once the hardware is attached (look for `TODO` markers).
- Topic names and node names match `QBOT_OVERVIEW.md` — keep them in sync if you
  change one.
- Nodes are written to **start cleanly without hardware** (e.g. the Kinect node
  runs in a no-device mode and warns instead of crashing), so the rest of the
  graph stays testable.
- Tune behavior in `src/behavior_node/config/behavior.yaml`; keep indoor speeds
  gentle and preserve the input watchdog (no runaway on stale data).

## Still to do (hardware-dependent)
- Kinect: build/verify libfreenect + firmware, confirm RGB+depth streams.
- Kobuki: add the base driver to `qbot.launch.py`, verify `/cmd_vel` moves it.
- Gesture classifier: implement real detection in `gesture_command_node`.
- Owner recognition: add to `face_tracker_node` (currently largest-face tracking).
- Depth-aware collision avoidance in `pet_behavior_node`.
