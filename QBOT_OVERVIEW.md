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
