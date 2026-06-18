# QBot — Progress & Context (for a fresh Raspberry Pi setup)

This is the **single source of truth** for picking the project up on a new
Raspberry Pi 5 (or after a wipe). It records what works, the exact bring-up
order, and every non-obvious gotcha discovered while getting the hardware live —
so you (or a future session) don't have to rediscover them.

Last updated: **2026-06-18**. See also: [`INSTALL.md`](INSTALL.md) (concise
install), [`COMMANDS.md`](COMMANDS.md) (command cheat-sheet), [`CLAUDE.md`](CLAUDE.md)
(architecture + conventions), [`QBOT_OVERVIEW.md`](QBOT_OVERVIEW.md),
[`team_flysky_project.md`](team_flysky_project.md).

---

## 1. Current status — what works

| Subsystem | Status |
|---|---|
| ROS 2 Jazzy + colcon workspace | ✅ builds clean |
| Kinect 360 RGB-D | ✅ RGB + depth streaming (libfreenect + Python binding built from source) |
| Kobuki base | ✅ driver built from source, connects (HW 1.0.4 / FW 1.1.4), drives, `/odom` + battery |
| Gesture recognition | ✅ **ONNX hand landmarks** (palm + 21 landmarks), all 7 gestures live-verified |
| Behavior maneuvers | ✅ closed-loop off `/odom` (stop/forward/backward/rotate360/turn/wag) |
| Kobuki buzzer cues | ✅ a tune per action |
| Full system bringup | ✅ `ros2 launch behavior_node qbot.launch.py` |

**Verified live on real hardware** (calibration + drive tests). The robot drives,
turns, spins, and responds to hand gestures with audio cues.

---

## 2. Hardware

- **Board:** Raspberry Pi 5, 4 GB RAM. Ubuntu 24.04 LTS (Noble), arm64.
- **Camera:** Xbox **Kinect v1 / 360** over USB. **Needs its external 12 V power
  adapter** (Y-cable) — on USB bus power alone only the *motor* enumerates; the
  camera/audio drop off the bus.
- **Base:** **Kobuki** (Yujin "iClebo Kobuki"), FTDI USB-serial → `/dev/kobuki`.
- This repo **is** the colcon workspace (a.k.a. `qbot_ws`), checked out at
  `~/flysky`.

---

## 3. Bring-up on a fresh Pi (the exact order)

> The one-shot installer `setup_qbot_env.sh` automates steps 2–6. This section
> explains what it does and the order that matters, so you can debug if a step
> fails.

### Step 1 — clone
```bash
cd ~ && git clone git@github.com:NadeeshaNJ/flysky.git && cd flysky
```

### Step 2 — system + ROS + deps (installer, needs sudo)
```bash
bash setup_qbot_env.sh        # asks for your sudo password; safe to re-run
```
It performs, in order:
1. **Locale**, then **enables the `noble-updates` apt pocket**. ⚠️ Some Pi images
   ship with only `noble` + `noble-security`; without `noble-updates`, ROS
   install fails with "held broken packages" (security-patched runtime libs
   outrun their `-dev` packages). This bit me — the installer now fixes it.
2. Adds the **ROS 2 apt source**, installs **`ros-jazzy-ros-base`** + dev tools
   (`colcon`, `cmake`, `rosdep`) — `ros-base`, not desktop, to stay light.
3. ROS deps for the nodes (`cv-bridge`, `image-transport`, OpenCV, NumPy …) and
   **Kobuki build deps** (`kobuki-ros-interfaces`, `kobuki-velocity-smoother`,
   `sophus`, `angles`, `tf2`, `diagnostic-updater`, `libftdi`, `libusb`, `eigen`,
   `cython3`, `python3-pip`).
4. **Kinect:** installs `libfreenect`, **blacklists the kernel `gspca_kinect`
   driver** (it grabs the camera and blocks libfreenect), adds you to
   `video`/`plugdev`/`dialout`, installs udev rules.
5. **Kobuki udev rule** → stable **`/dev/kobuki`** symlink (matches the "iClebo
   Kobuki" product string, not just any FTDI 0403:6001).
6. **Builds the libfreenect Python binding** (`import freenect`) from source —
   it is **not** in apt. Installs to system `dist-packages`.
7. **Clones + patches the Kobuki driver stack source** (`kobuki_core`,
   `kobuki_ros`, full `ecl` library — none in apt for Jazzy) into `src/`. Patches
   `ecl`'s `-Werror` out (it fails on Ubuntu 24.04's GCC), and `COLCON_IGNORE`s
   packages we don't need (`kobuki_keyop` needs the unpackaged `cmd_vel_mux`).
8. **Gestures:** `pip install onnxruntime` and downloads the **ONNX hand-landmark
   models** into `src/gesture_node/models/`.

### Step 3 — log out / back in
So the `video`/`plugdev`/`dialout` group membership applies.

### Step 4 — add swap (4 GB Pi OOMs building the C++ ecl/Kobuki stack)
```bash
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile \
  && sudo mkswap /swapfile && sudo swapon /swapfile
```

### Step 5 — build (limited parallelism, ~15–20 min first time)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/flysky
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Step 6 — verify hardware
```bash
# Kinect (expect Motor + Audio + Camera)
lsusb | grep 045e
python3 -c "import freenect,time; t=time.time(); v=d=None
while time.time()-t<6 and (v is None or d is None): v=freenect.sync_get_video(); d=freenect.sync_get_depth()
freenect.sync_stop(); print('Kinect', 'OK' if v and d else 'FAIL')"
# Kobuki (prints a HW/FW version line; Ctrl-C to stop)
ls -l /dev/kobuki
ros2 launch kobuki_node kobuki_node-launch.py
```

### Step 7 — run
```bash
ros2 launch behavior_node qbot.launch.py                 # full system, robot live
ros2 launch behavior_node qbot.launch.py use_base:=false # perception only (no motion)
ros2 launch behavior_node qbot.launch.py linear_speed:=0.15 turn_speed:=1.3  # speeds
```

---

## 4. Architecture

```
Kinect ──RGB──> gesture_command_node ──/gesture/tracking──> pet_behavior_node ──/commands/velocity──> Kobuki
       └─depth─> (gesture uses RGB; depth available for future collision avoidance)
                                                            └──/commands/sound (buzzer)──> Kobuki
Kobuki ──/odom──────────────────────────────────────────> pet_behavior_node (closed-loop maneuvers)
```

| Package | Node | Key I/O |
|---|---|---|
| `kinect_camera` | `kinect_rgbd` | → `/kinect/rgb/image_raw`, `/kinect/depth/image_raw` |
| `vision_node` | `face_tracker_node` | → `/vision/target` (⚠️ **off by default** — see §7) |
| `gesture_node` | `gesture_command_node` | RGB → `/gesture/tracking` (String) |
| `behavior_node` | `pet_behavior_node` | gesture + `/odom` → `/commands/velocity`, `/commands/sound` |

**Kobuki specifics:** velocity input is **`/commands/velocity`** (Twist), *not*
`/cmd_vel`. Its `/commands/sound` is a typed `kobuki Sound` msg (built-in buzzer
tunes). Abstract behavior cues also go to `/qbot/sound` (String).

---

## 5. Gesture system (the important part)

**Why ONNX, not MediaPipe or depth:**
- MediaPipe has **no ARM64 wheel** → can't install on the Pi.
- A depth-silhouette approach was tried first and **abandoned**: at arm's length
  the Kinect depth blob can't separate an open palm from a fist (solidity ~0.80
  vs ~0.91 — too close), and finger-counting via convexity defects flickers.
- **`onnxruntime` *does* have an ARM64 wheel.** So we run OpenCV model-zoo's
  converted MediaPipe models (palm detection + 21 hand landmarks) under
  onnxruntime. Live result: **~100 % detection at 0.98+ confidence**, reliable
  per-finger states. This was the single biggest quality jump.

**Pipeline:** RGB → `mp_models/` (palm detect → 21 landmarks, onnxruntime) →
`hand_tracker.py` (landmarks → finger states/centroid) → `gesture_classifier.py`
(temporal state machine → command event, rate-capped, with cooldown).

**Vocabulary (all live-verified):**
| Sign | Command | Robot |
|---|---|---|
| Open palm held | `stop` | halt now (interrupts any maneuver) |
| Curl hand twice (open↔fist) | `forward` | drive forward until stopped |
| **Thumbs-down** held | `backward` | drive backward until stopped |
| Index finger drawing a circle | `rotate360` | spin 360° |
| Open-hand swipe left / right | `turn_left` / `turn_right` | sidestep + turn back to face you |
| Rapid open-hand wave | `tail_wag` | wag 3× |

**Calibration notes (hard-won):**
- The hand must be the prominent hand in frame; works ~50 cm from the camera.
- A **wrapped thumb miscounts as 1 finger**, so a fist reads `fingers==1` —
  `is_closed` accepts ≤1 finger when *not* pointing (`index_only`).
- The **rotate circle** is detected from the fingertip's *absolute* trajectory
  (works whether the finger pivots or the whole hand moves in a circle).
- **Turns** require a deliberate, predominantly-horizontal swipe (`|dx|>1.8|dy|`,
  `>= swipe_dx`, low reversals) — otherwise incidental drift triggers them. The
  left/right mapping is **swapped** because the camera image is mirrored vs you.
- `stop` bypasses the post-gesture cooldown so it can abort a running maneuver.
- Debug: `ros2 run gesture_node gesture_command_node --ros-args -p debug:=true`
  logs `fingers / index_only / thumb_down / cx / cy`. Tune `swipe_dx`,
  `process_hz`, and classifier thresholds to your hand/lighting.

---

## 6. Known issues / gotchas (read before debugging)

- **Kinect USB fragility.** The libfreenect *sync* API leaves the USB device
  **claimed** if a process is hard-killed without `freenect.sync_stop()`. Always
  stop nodes with **Ctrl-C / SIGINT** (the node calls `sync_stop()` in `finally`).
  Never `kill -9` the Kinect node. If the camera/audio drop off `lsusb` (only the
  motor left), recover with `sudo python3 scripts/kinect_usb_reset.py` — but note
  the motor reset itself sometimes drops the camera and it takes a retry to come
  back. **A physical USB unplug/replug is the most reliable recovery** and also
  restores full frame rate.
- **Frame-rate vs CPU.** Standalone the Kinect node hits ~30 Hz; with the ONNX
  gesture node running, the camera loop is CPU-starved. The **face tracker is off
  by default** because nothing consumes `/vision/target` yet and it cost ~3× the
  frame rate (5 → 15 Hz when dropped). Re-enable with `use_face:=true` only once
  something needs it.
- **Don't `pkill -f <nodename>`** in a shell whose own command line contains that
  name — it matches and kills your shell. Use `pgrep -f '[n]ame'` (bracket trick).
- **Kobuki "Malformed sub-payload detected"** appears occasionally on the serial
  link; usually transient and the driver recovers. A clean restart clears it.
- **apt "held broken packages"** installing ROS → `noble-updates` not enabled
  (installer fixes it; `sudo apt update` after).

---

## 7. What's left / next steps

- **Owner recognition** in `face_tracker_node` (currently largest-face; and it's
  off by default — wire a consumer + turn it on).
- **Depth-aware collision avoidance** in `pet_behavior_node` (depth stream is
  already published, unused).
- **On-hand gesture tuning** per user/lighting (thresholds are exposed as params).
- Optional: MediaPipe-style hand *tracking* (skip palm detection between frames)
  to raise gesture FPS.

---

## 8. File map (where things live)

```
setup_qbot_env.sh          one-shot installer (idempotent)
scripts/
  kinect_usb_reset.py      recover a stuck/dropped Kinect (USBDEVFS_RESET)
  kobuki_drive_test.py     gentle live drive test (run kobuki driver first)
  sim_behavior_test.py     maneuver logic in sim (no robot needed)
src/
  kinect_camera/           kinect_rgbd  (libfreenect Python binding)
  vision_node/             face_tracker_node (OpenCV Haar; off by default)
  gesture_node/
    hand_tracker.py        ONNX landmarks -> finger states
    gesture_classifier.py  temporal state machine -> commands
    gesture_command_node.py
    mp_models/             vendored OpenCV-zoo MPPalmDet/MPHandPose (onnxruntime)
    models/                *.onnx weights (git-ignored, downloaded by installer)
  behavior_node/
    pet_behavior_node.py   odom closed-loop maneuver executor + buzzer
    launch/qbot.launch.py  full-system bringup (use_base/use_kinect/use_face/speeds)
    config/behavior.yaml
  ecl_*/ kobuki_core/ kobuki_ros/   third-party, git-ignored, cloned by installer
```

Everything git-ignored (build/install/log, third-party source, ONNX weights, the
freenect firmware) is regenerated by `setup_qbot_env.sh` + `colcon build`.
