# QBot — Installation Guide

Setup instructions for the **FlySky QBot** on a **Raspberry Pi 5 (4 GB)** running
**Ubuntu 24.04 LTS (arm64)** with **ROS 2 Jazzy**.

---

## 1. Hardware

| Part | Notes |
|---|---|
| Raspberry Pi 5 (4 GB) | Ubuntu 24.04 LTS, arm64 |
| Xbox **Kinect 360 / v1** | **Must** use its external **12 V power adapter** (Y-cable). USB bus power alone only runs the motor — the camera/audio drop off the bus without it. |
| **Kobuki** mobile base | Connects via its USB cable (FTDI serial) |
| microSD ≥ 16 GB | ~5 GB used after install |

Plug in both devices before running the installer so udev rules and group
membership apply to them.

---

## 2. One-command install

From the repository root:

```bash
bash setup_qbot_env.sh
```

You will be prompted for your **sudo password** (the script installs system
packages and udev rules). It is safe to re-run. It performs:

1. Locale + the **`noble-updates`** apt pocket (some Pi images ship without it,
   which breaks ROS dependency resolution).
2. **ROS 2 Jazzy** (`ros-base`) + build tools (`colcon`, `cmake`, `rosdep`).
3. ROS deps for the QBot nodes (`cv_bridge`, `image_transport`, OpenCV, NumPy …).
4. **Kobuki** build deps (`kobuki-ros-interfaces`, `kobuki-velocity-smoother`,
   `sophus`, `angles`, `tf2`, `diagnostic-updater`, `libftdi`, `libusb`, `eigen`).
5. **Kinect**: `libfreenect`, builds the **`freenect` Python binding** from source
   (not in apt), blacklists the conflicting `gspca_kinect` kernel driver, installs
   udev rules, and adds you to `video`/`plugdev`/`dialout`.
6. **Kobuki** udev rule → stable **`/dev/kobuki`** symlink.
7. Clones + patches the **Kobuki driver stack** source (`kobuki_core`,
   `kobuki_ros`, and the full `ecl` library — none are in apt for Jazzy) into
   `src/`. The `ecl` `-Werror` is patched out (it fails on Ubuntu 24.04's GCC).
8. **Gesture recognition**: installs `onnxruntime` (pip) and downloads the ONNX
   hand-landmark models (palm detection + 21 landmarks) into
   `src/gesture_node/models/`. MediaPipe has no ARM64 wheel; onnxruntime does.
9. Auto-sources ROS in your `~/.bashrc`.

After it finishes, **log out and back in** (or `newgrp plugdev`) so the new group
membership takes effect.

---

## 3. Build the workspace

The Pi has limited RAM — build with bounded parallelism (and add swap first if you
have none):

```bash
# optional: 2 GB swap to avoid OOM during the C++ build
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile \
  && sudo mkswap /swapfile && sudo swapon /swapfile

source /opt/ros/jazzy/setup.bash
cd ~/flysky                 # the workspace root (this repo)
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

First build takes ~15–20 min (it compiles the `ecl` + Kobuki C++ stack). Re-builds
are fast.

---

## 4. Verify the hardware

```bash
# Kinect: grab a few RGB+depth frames
python3 -c "import freenect,time; t=time.time();
v=d=None
while time.time()-t<5 and (v is None or d is None): v=freenect.sync_get_video(); d=freenect.sync_get_depth()
freenect.sync_stop(); print('Kinect OK' if v and d else 'Kinect FAIL')"

# Kobuki: should print a HW/FW version line, then Ctrl-C
ros2 launch kobuki_node kobuki_node-launch.py
```

---

## 5. Run

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Full system (camera + perception + base). Put the robot on the floor, clear space.
ros2 launch behavior_node qbot.launch.py

# Perception only, no robot motion (safe for desk testing / gesture calibration):
ros2 launch behavior_node qbot.launch.py use_base:=false
```

Gesture vocabulary and behaviors are documented in
[`CLAUDE.md`](CLAUDE.md#gesture-control-implemented). To calibrate gesture
detection, run the gesture node with debug logging:

```bash
ros2 run gesture_node gesture_command_node --ros-args -p debug:=true
```

---

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| Kinect camera/audio vanish from `lsusb` (only motor left) | Check the **12 V adapter**; recover with `sudo python3 scripts/kinect_usb_reset.py` |
| `Could not open device` / `subdevice not disabled` | A node was hard-killed and left the USB claimed. Always stop nodes with **Ctrl-C** (never `kill -9`). Then run the reset script above. |
| Kinect camera not opening at all | Ensure `gspca_kinect` is blacklisted (`/etc/modprobe.d/blacklist-kinect.conf`); reboot or `sudo modprobe -r gspca_kinect` |
| `colcon build` killed / OOM | Add swap (see §3) and use `--parallel-workers 1 MAKEFLAGS="-j2"` |
| apt "held broken packages" installing ROS | Enable `noble-updates` (the installer does this) and `sudo apt update` |
| Kobuki: `no device port` | Use the launch file (`kobuki_node-launch.py`), not the bare executable; check `/dev/kobuki` exists |
| Gestures not recognised | Check the ONNX models exist in `src/gesture_node/models/` and `onnxruntime` imports; run the node with `-p debug:=true` to watch `fingers/index_only/cx`; hold your hand ~50 cm from the camera; tune `swipe_dx` / `process_hz` |
| `ModuleNotFoundError: onnxruntime` | `python3 -m pip install --break-system-packages onnxruntime` |

---

## 7. Reset / reinstall

The third-party source (`src/ecl_*`, `src/kobuki_core`, `src/kobuki_ros`) and the
`build/ install/ log/` dirs are git-ignored. To start clean:

```bash
rm -rf build install log src/ecl_* src/kobuki_core src/kobuki_ros
bash setup_qbot_env.sh    # re-clones and re-patches the source
```
