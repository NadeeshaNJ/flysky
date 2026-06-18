# Gesture Algorithm Reference for FlySky

This file documents the gesture logic that works in `final-project-theflysky` so
the newer `flysky` repo can port the same behavior even though its hand detector
uses ONNX MediaPipe models instead of the MediaPipe Tasks `GestureRecognizer`.

Main source files in this repo:

- `qbot_ws/src/gesture_node/gesture_node/gesture_command_node.py`
- `qbot_ws/src/gesture_node/gesture_node/signals.py`
- `qbot_ws/src/gesture_node/gesture_node/frame_processor.py`

Equivalent files in the `flysky` repo:

- `src/gesture_node/gesture_node/hand_tracker.py`
- `src/gesture_node/gesture_node/gesture_classifier.py`
- `src/gesture_node/gesture_node/gesture_command_node.py`

## Core Idea

The working algorithm is not "run recognizer, publish label". The reliable part
is this sequence:

1. Decode Kinect RGB frame.
2. Reject frames that are too dark or saturated.
3. Apply CLAHE to improve luma contrast.
4. Downscale to a fixed width for speed and consistent thresholds.
5. Detect one hand and get 21 normalized landmarks.
6. Smooth landmark positions with EMA and reject large jumps.
7. Derive stable per-frame features:
   - coarse hand label
   - finger pattern
   - hand openness
   - hand center
   - index fingertip position
   - pointing direction
8. Majority-vote labels across a short history.
9. Feed the feature stream into temporal detectors.
10. Publish a command only when the temporal pattern is confirmed.

For ONNX, steps 1-5 already exist in `flysky/hand_tracker.py`. The part to copy
is mostly steps 6-10.

## Landmark Contract

The algorithm assumes MediaPipe's 21 hand landmark order:

| Index | Meaning |
| --- | --- |
| 0 | wrist |
| 4 | thumb tip |
| 8 | index tip |
| 12 | middle tip |
| 16 | ring tip |
| 20 | pinky tip |
| 5 | index MCP |
| 17 | pinky MCP |
| 6,10,14,18 | finger PIP joints |

MediaPipe Tasks returns normalized coordinates in `[0..1]`. The current ONNX
wrapper in `flysky` returns pixel coordinates and then converts some values to
`[-1..1]`. Either is fine, but do not mix thresholds between coordinate systems.

Recommended porting choice:

- In `hand_tracker.py`, keep the raw 21 landmarks.
- Normalize landmark `x` by image width and `y` by image height.
- Let the classifier consume normalized landmarks, not only `fingers/cx/cy`.

## Per-Frame Features

### Finger Pattern

Return `[thumb, index, middle, ring, pinky]`, where `1` means extended.

Final project logic:

- Thumb is up only when both are true:
  - thumb IP angle is greater than `150 deg`
  - thumb tip is farther from wrist than thumb IP by about `0.035` normalized units
- Index/middle/ring/pinky are up when `tip.y < pip.y`.

This is better than the current ONNX count-only approach because it preserves
which finger is up. The distinction between `index_only` and `fist with thumb
noise` matters for rotate and point commands.

Pattern labels:

| Fingers | Label |
| --- | --- |
| `[0,0,0,0,0]` | `FIST` |
| `[1,1,1,1,1]` | `OPEN_PALM` |
| `[0,1,0,0,0]` | `POINTING` |
| `[1,0,0,0,0]` | `THUMBS_UP` |
| `[0,1,1,0,0]` | `PEACE` |
| `[1,1,0,0,1]` | `I_LOVE_YOU` |

For the robot commands, the important labels are `OPEN_PALM`, `POINTING`,
`FIST`, and `THUMBS_DOWN`.

### Hand Openness

Do not use only MediaPipe's `Open_Palm` / `Closed_Fist` labels for beckon. The
working project uses a continuous openness value:

```text
palm_width = distance(landmark 5, landmark 17)
mean_tip_distance = mean(distance(wrist, landmarks 8,12,16,20))
ratio = mean_tip_distance / palm_width
openness = clamp((ratio - 1.0) / 1.6, 0.0, 1.0)
```

Interpretation:

- `openness >= 0.70`: open palm for stop/wave tracking
- `openness >= 0.62`: open state for beckon hysteresis
- `openness <= 0.32`: closed state for beckon hysteresis
- `openness < 0.55`: clear open-palm wave buffer

This is the most important part to port. It makes beckon/curl work even when
the recognizer produces unstable labels during a half-curled hand.

### Hand Center and Span

Use all 21 landmarks:

```text
center_x = mean(all landmark x)
center_y = mean(all landmark y)
span = max(max_x - min_x, max_y - min_y)
```

Reject tiny hands/noisy detections:

```text
if span < 0.055: treat as no hand
```

For ONNX pixel landmarks, normalize before using `0.055`.

### Pointing Direction

Use index MCP to index tip:

```text
dx = tip.x - mcp.x
dy = tip.y - mcp.y
dz = mcp.z - tip.z
```

Rules:

- If `dz > 0.12` and image-plane length is below `0.10`, direction is
  `TOWARDS_CAMERA`.
- If horizontal motion dominates, use `LEFT` / `RIGHT`:
  - `abs(dx) > max(0.06, abs(dy) * 1.25)`
- Otherwise classify the vector into 8 compass sectors.

Only `LEFT` and `RIGHT` trigger movement commands.

## Stability Filters

### Landmark Smoother

Use exponential moving average with jump rejection:

```text
alpha = 0.55
jump_threshold = 0.22

if landmark jump > jump_threshold:
    blend = 0.20
else:
    blend = alpha

smoothed = blend * current + (1 - blend) * previous
```

This prevents one bad frame from corrupting a circle, wave, or point hold.

### Majority Label Filter

Keep the last `7` raw labels and switch to a label only when it gets `4` votes.
This avoids rapid `CUSTOM`/`POINTING`/`OPEN_PALM` flicker.

## Temporal Command Decoder

The decoder keeps sliding buffers:

- open palm track: `(time, center_x, center_y)`
- point track: `(time, index_tip_x, index_tip_y)`
- beckon openness detector state
- held-state timestamps for stop, point left/right, thumbs-down
- per-command cooldown timestamps

The final project evaluates commands in this order:

1. `TAIL_WAG`
2. `COME_CLOSER`
3. `ROTATE_ONCE`
4. `MOVE_LEFT_FOOT` / `MOVE_RIGHT_FOOT`
5. `MOVE_BACK`
6. `STOP`

In the current `flysky` repo, `stop` is intentionally checked before cooldown so
it can abort movement. Keep that behavior if robot safety depends on it. The
temporal math below is the important part.

### COME_CLOSER: Beckon / Curl Twice

Use `BeckonOscillationDetector`, not label transitions.

Parameters:

```text
oscillations_required = 2
window_sec = 4.5
open_threshold = 0.62
closed_threshold = 0.32
min_half_period_sec = 0.18
cooldown = 2.4
```

Algorithm:

1. Update with every frame's `openness`.
2. State becomes `open` when `openness >= 0.62`.
3. State becomes `closed` when `openness <= 0.32`.
4. Count one curl when state changes from `closed` to `open` after the minimum
   half-period.
5. If two curls occur within `4.5s`, emit `COME_CLOSER` / `forward`.

For `flysky`, map this command to the existing `forward` behavior.

### STOP: Open Palm Held Still

Parameters:

```text
openness >= 0.70
hold_sec = 0.45
motion_tolerance = 0.04
cooldown = 0.9
```

Algorithm:

1. Start `open_palm_since` when `openness >= 0.70`.
2. Append hand center to palm track.
3. After `0.45s`, inspect recent `0.5s` of palm centers.
4. If horizontal range is greater than `motion_tolerance * 3`, do not stop
   because it is probably a wave.
5. Otherwise emit `STOP` / `stop`.

### TAIL_WAG: Open Palm Wave

Parameters:

```text
openness >= 0.70
window_sec = 2.4
min_samples = 8
palm_wave_amp = 0.08
palm_wave_sweep = 0.18
required_reversals = 3
cooldown = 2.8
```

Algorithm:

1. Track `center_x` only while the hand is open.
2. Require at least 8 samples.
3. Require `max(x) - min(x) >= palm_wave_amp * 2`.
4. Count direction reversals with minimum sweep `palm_wave_sweep / 2.4`.
5. If reversals are at least 3, emit `TAIL_WAG` / `tail_wag`.

The reversal counter ignores tiny movements until the hand moves by the minimum
sweep, then counts only meaningful direction changes.

### ROTATE_ONCE: Index Finger Circle

Parameters:

```text
gesture = POINTING
window_sec = 3.2
min_samples = 12
min_duration = 0.55
min_mean_radius = 0.04
max_radius_std = mean_radius * 0.95
min_net_angle = 1.55 * pi
min_total_angle = 1.75 * pi
required_quadrants = 4
cooldown = 3.0
```

Algorithm:

1. Track index fingertip while stable label is `POINTING`.
2. Compute center of the fingertip trajectory.
3. Compute radius of every point from that trajectory center.
4. Reject tiny jitter and wildly inconsistent radius.
5. Unwrap `atan2(y - cy, x - cx)` angles.
6. Require enough net rotation, enough total angular motion, and coverage of all
   four quadrants.
7. Emit `ROTATE_ONCE` / `rotate360`.

This works whether the user rotates only the finger or moves the whole hand in a
circle, because the circle is measured around the fingertip trajectory center.

### MOVE_LEFT_FOOT / MOVE_RIGHT_FOOT: Index Held Left/Right

Parameters:

```text
gesture = POINTING
direction in LEFT/RIGHT
hold_sec = 0.55
cooldown = 2.2
```

Algorithm:

1. When stable label is `POINTING`, compute pointing direction.
2. If direction changes, restart the hold timer.
3. If the same `LEFT` or `RIGHT` direction is held for `0.55s`, emit the
   matching command.

For `flysky`, map these to `turn_left` / `turn_right` if that is the behavior
API currently implemented.

### MOVE_BACK: Thumbs Down Held

Parameters:

```text
gesture = THUMBS_DOWN
hold_sec = 0.55
cooldown = 2.2
```

The final project uses MediaPipe's `Thumb_Down` label. The current ONNX FlySky
repo approximates this in `hand_tracker.py`:

```text
four fingers curled AND thumb_tip.y - thumb_mcp.y > 18 pixels
```

If porting into normalized landmarks, replace `18 pixels` with a scale-aware
threshold, for example:

```text
thumb_tip.y - thumb_mcp.y > 0.08
```

Then hold that state for `0.55s` before emitting `MOVE_BACK` / `backward`.

## MediaPipe to ONNX Mapping

The final project uses MediaPipe Tasks:

```text
GestureRecognizer -> gestures + hand_landmarks
```

The FlySky repo uses:

```text
ONNX palm detector -> ONNX hand pose -> 21 landmarks
```

The temporal algorithm does not require MediaPipe Tasks. It only needs:

```text
hand_visible
21 normalized landmarks
confidence
timestamp
```

Therefore, in FlySky:

1. Modify `HandFeatures` to include `landmarks`.
2. Normalize ONNX screen landmarks:

   ```python
   norm_lms = [(x / w, y / h, z_norm) for x, y, z in lms]
   ```

3. Port `hand_openness`, `LandmarkSmoother`, `MajorityLabelFilter`,
   `count_axis_reversals`, and `BeckonOscillationDetector` from `signals.py`.
4. Replace count-only logic in `gesture_classifier.py` with the temporal decoder
   rules above.
5. Keep existing FlySky command names if the behavior node expects them:

   | Final project command | FlySky command |
   | --- | --- |
   | `COME_CLOSER` | `forward` |
   | `STOP` | `stop` |
   | `ROTATE_ONCE` | `rotate360` |
   | `MOVE_LEFT_FOOT` | `turn_left` |
   | `MOVE_RIGHT_FOOT` | `turn_right` |
   | `TAIL_WAG` | `tail_wag` |
   | `MOVE_BACK` | `backward` |

## Suggested FlySky Refactor

Minimal change path:

1. In `hand_tracker.py`, return both old fields and normalized `landmarks`.
2. Add a new helper module, for example `gesture_node/signals.py`, copied from
   this final project.
3. In `gesture_classifier.py`, compute per-frame features from landmarks:
   `fingers`, `label`, `openness`, `center`, `index_tip`, `pointing_direction`.
4. Use the final project's temporal detector thresholds, then convert emitted
   names to the existing FlySky lowercase command strings.
5. Run with debug logging and print:

   ```text
   label raw_label fingers openness center_x center_y pointing_direction
   ```

## Tuning Notes

- If beckon does not fire, log `openness`. A fully open hand should be above
  `0.62`, and a curled/fist hand should drop below `0.32`.
- If stop fires during a wave, increase `stop_motion_tolerance` or evaluate
  `TAIL_WAG` before `STOP`.
- If wave does not fire, lower `palm_wave_amp` from `0.08` to `0.06`.
- If circle fires from jitter, increase `min_mean_radius` from `0.04` to `0.06`.
- If point left/right is mirrored, flip `mirror_horizontal_commands`.
- If the ONNX landmarks are pixel-based, all thresholds above must be applied
  after normalizing to `[0..1]`.

## Quick Pseudocode

```python
frame = decode_ros_image(msg, target="rgb")
if not assess_quality(frame).accepted:
    publish_no_hand()
    return

frame = apply_clahe(frame)
frame = downscale(frame, 480)
landmarks = onnx_hand_landmarks(frame)  # normalize to [0..1]

if not landmarks:
    decoder.reset()
    publish_no_hand()
    return

landmarks = smoother.update(landmarks)
center_x, center_y, span = hand_center_and_span(landmarks)
if span < 0.055:
    decoder.reset()
    publish_no_hand()
    return

fingers = fingers_up(landmarks)
pattern_label = label_from_finger_pattern(fingers)
openness = hand_openness(landmarks)
stable_label = majority_filter.update(pattern_label)

obs = {
    "time": monotonic_time,
    "hand_visible": True,
    "gesture": stable_label,
    "center_x": center_x,
    "center_y": center_y,
    "span": span,
    "openness": openness,
    "index_tip": (landmarks[8].x, landmarks[8].y),
    "pointing_direction": pointing_direction(landmarks),
    "fingers": fingers,
}

cmd = decoder.update(obs)
if cmd:
    publish(command_name_map[cmd])
```

