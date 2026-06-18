# QBot — Command Reference

A cheat-sheet of the commands used to build, run, test, and operate QBot.
See [`INSTALL.md`](INSTALL.md) for first-time setup and [`CLAUDE.md`](CLAUDE.md)
for architecture + the gesture vocabulary.

> Almost every session starts by sourcing ROS and the workspace:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/flysky/install/setup.bash
> ```

---

## Live gesture test — the short version

Three commands, one terminal each (source ROS + workspace first, as above).

```bash
# 1. (Only if the camera is stuck — no frames / "Could not open audio") reset USB:
sudo python3 scripts/kinect_usb_reset.py

# 2. Camera + gesture recognizer together, with debug logging:
ros2 launch gesture_node gesture_test.launch.py

# 3. Watch the commands it emits (optional, separate terminal):
ros2 topic echo /gesture/tracking
```

Stop with a single **Ctrl-C** in the launch terminal — it shuts the Kinect down
cleanly. **Never `kill -9`** the camera: libfreenect leaves the USB device
claimed, and you'll have to run the reset in step 1.

> Sanity check the camera is actually streaming before making gestures:
> ```bash
> ros2 topic hz /kinect/rgb/image_raw --qos-reliability best_effort   # expect ~30
> ```
> If it prints nothing, the Kinect isn't sending frames — check the **12V power
> adapter (Y-cable)** is connected, then re-run the reset (step 1).

---

## Build

```bash
cd ~/flysky
# Full build (limit parallelism on the 4 GB Pi to avoid OOM)
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Rebuild just one package (fast)
colcon build --symlink-install --packages-select gesture_node
colcon build --symlink-install --packages-select behavior_node
```

> With `--symlink-install`, edits to Python files take effect on the next node
> start — no rebuild needed. Rebuild is only needed for new files, package.xml,
> setup.py, or C++ changes.

---

## Run the full system

```bash
# Everything: Kobuki base + Kinect + face tracker + gesture + behavior
ros2 launch behavior_node qbot.launch.py

# Perception only, robot does NOT move (safe for gesture testing)
ros2 launch behavior_node qbot.launch.py use_base:=false

# No camera (e.g. replaying a bag)
ros2 launch behavior_node qbot.launch.py use_kinect:=false

# Gentle speeds for a first/supervised run
ros2 launch behavior_node qbot.launch.py
# ...or run the behavior node alone with overrides:
ros2 run behavior_node pet_behavior_node --ros-args \
  -p cmd_vel_topic:=/commands/velocity -p linear_speed:=0.08 -p turn_speed:=0.6
```

## Run individual nodes

```bash
ros2 run kinect_camera  kinect_rgbd
ros2 run vision_node    face_tracker_node
ros2 run gesture_node   gesture_command_node                 # gesture recognizer
ros2 run gesture_node   gesture_command_node --ros-args -p debug:=true   # + feature logging
ros2 run behavior_node  pet_behavior_node --ros-args -p cmd_vel_topic:=/commands/velocity
ros2 launch kobuki_node kobuki_node-launch.py                # Kobuki driver alone
ros2 launch kinect_camera kinect_rgbd.launch.py              # Kinect alone
```

---

## Inspect / debug at runtime

```bash
ros2 node list
ros2 topic list
ros2 topic echo /gesture/tracking          # gesture command events
ros2 topic echo /vision/target             # detected face (PointStamped)
ros2 topic echo /commands/velocity         # velocity sent to the base
ros2 topic hz   /kinect/rgb/image_raw      # camera frame rate
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /sensors/core --field battery   # Kobuki battery (decivolts: 154 = 15.4 V)
ros2 param list /pet_behavior_node
```

### Key topics
| Topic | Type | Who |
|---|---|---|
| `/kinect/rgb/image_raw`, `/kinect/depth/image_raw` | Image | kinect_rgbd → |
| `/vision/target` | PointStamped | face_tracker → |
| `/gesture/tracking` | String | gesture_command → |
| `/commands/velocity` | Twist | behavior → Kobuki |
| `/qbot/sound` | String | behavior (audio cue) |
| `/odom`, `/sensors/core` | Odometry / sensor | Kobuki → |

---

## Hardware tests / recovery

```bash
# Kinect: recover a stuck/dropped device (camera/audio gone from lsusb)
sudo python3 scripts/kinect_usb_reset.py
lsusb | grep 045e                          # expect Motor + Audio + Camera
freenect-camtest                           # raw stream test (Ctrl-C to stop)

# Kobuki: confirm the base link + a gentle live nudge (robot must be on the floor)
ls -l /dev/kobuki
python3 scripts/kobuki_drive_test.py       # run `ros2 launch kobuki_node kobuki_node-launch.py` first

# Maneuver logic in sim (no robot needed) — run pet_behavior_node first
python3 scripts/sim_behavior_test.py
```

---

## Gesture calibration

```bash
# Watch what the recognizer sees while you make gestures (the launch above
# already sets debug:=true; this is the node on its own if the camera's running)
ros2 run gesture_node gesture_command_node --ros-args -p debug:=true
# logs: hand: label=.. fingers=[t,i,m,r,p] openness=.. point=.. nx=.. ny=.. span=..
#       and   gesture recognised -> X

# Tunables (ROS params on gesture_command_node)
#   process_hz          inference rate cap (default 12)
#   mirror_horizontal   flip turn_left/right if pointing reads mirrored (default true)
#   score_threshold / conf_threshold   palm / landmark model thresholds
# Finer thresholds (openness cutoffs, hold times, circle geometry) live in
# gesture_classifier.py — see GESTURE_ALGORITHM_REFERENCE.md for what each does.
```

The debug log is the calibration tool: a flat open palm should read
`openness` ≳ 0.7 and a fist ≲ 0.3. If a gesture won't fire, watch these values
while you perform it.

### Gesture vocabulary
| Sign | Command | Robot |
|---|---|---|
| Open palm held still | stop | halt now (works even mid-maneuver) |
| Curl hand twice (open↔fist ×2) | forward | drive forward until stop |
| Closed fist held ~1 s | backward | drive backward until stop |
| Index finger drawing a circle | rotate360 | spin 360° |
| Index finger pointed & held left / right | turn_left / turn_right | sidestep + face you |
| Rapid open-hand side-to-side wave | tail_wag | wag 3× |
| (face seen, idle) | — | wiggle every ~10 s |

---

## Stop everything

```bash
# In the launch terminal: Ctrl-C  (never kill -9 the Kinect node — it leaves the
# USB device claimed; if that happens, run scripts/kinect_usb_reset.py)
```
