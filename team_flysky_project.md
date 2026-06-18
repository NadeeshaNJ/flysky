# Interactive Companion Robot with Owner Recognition and Gesture Control

## Group Name

**FlySky**

## Abstract

This project aims to develop a pet-like companion robot capable of responding to simple hand gestures. The system uses a **Kinect 360 RGB camera** and computer vision techniques implemented with **OpenCV**. The robot will perform predefined actions such as **stop**, **spin**, **approach**, and **wiggle**.

---

## Introduction

Human-robot interaction is an important aspect of modern robotics, especially in service and companion robot applications.

In this project, the **Quanser QBot** will respond to simple gesture commands and interact with registered users while operating safely in an indoor environment.

---

## Methodology

The system will be implemented on a **Quanser QBot** using **ROS2**, with an RGB camera as the primary perception sensor. The software will follow a modular ROS2 architecture, where separate nodes handle:

- Image processing
- User recognition
- Behavior control
- Robot motion

A vision-processing node will use **OpenCV-based algorithms** to detect faces and hand gestures.

Once a person is identified, a behavior-control node will trigger companion-like responses such as:

- Turning toward the person
- Performing a small wiggle motion
- Moving slightly closer

Gesture commands will also be used to control simple interactions. If motion is involved, an obstacle-awareness component will ensure the robot avoids unsafe movement in its surroundings.

---

## Expected Outcomes

The system is expected to demonstrate a robot capable of recognizing people and responding with simple behaviors. It should detect faces and react to predefined gesture commands. The robot will also show safe navigation when movement is involved.

---

## Project Timeline

| Week | Task |
|---|---|
| **Week 1 & 2** | Set up the ROS2 workspace, configure the Kobuki robot, and connect the RGB camera. Verify basic communication between the robot and the camera. |
| **Week 3** | Develop the vision pipeline and implement basic face detection using the camera input. |
| **Week 4** | Develop the gesture detection module to recognize a small set of predefined hand gestures. |
| **Week 5** | Implement robot behaviors triggered by user recognition and gesture commands. |
| **Week 6** | Integrate all modules and test the complete system in an indoor environment to demonstrate the working prototype. |

---

## Individual Task Allocation

| Week | Ishan | Nadeesha |
|---|---|---|
| **Week 1 & 2** | Set up ROS2 workspace and install required ROS2 packages. Configure Kobuki drivers and verify robot movement commands. | Connect and configure the RGB camera. Set up camera drivers and verify image streaming in ROS2. |
| **Week 3** | Implement the vision pipeline in ROS2 and integrate OpenCV for image processing. | Implement face detection using OpenCV and test detection on camera frames. |
| **Week 4** | Develop the gesture detection algorithm and integrate it into the vision pipeline. | Collect gesture samples and test recognition accuracy for predefined gestures. |
| **Week 5** | Connect gesture recognition outputs to robot motion commands and test behavior responses. | Implement robot behavior logic that reacts to recognized users and gesture commands. |
| **Week 6** | Integrate all ROS2 nodes and ensure communication between modules works correctly. | Perform system testing in an indoor environment and evaluate recognition, gestures, and robot interaction. |

---

## References

- Kobuki / QBot Platform Documentation: <https://kobuki.readthedocs.io/en/devel/>
- OpenCV Documentation: <https://docs.opencv.org/4.x/>
