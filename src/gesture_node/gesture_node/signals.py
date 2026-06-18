"""Per-frame hand features and stability filters (ported from the laptop
MediaPipe project; see GESTURE_ALGORITHM_REFERENCE.md).

Everything here operates on **normalised** landmarks: an (21, 3) array with
``x`` in [0, 1] (fraction of image width), ``y`` in [0, 1] (fraction of image
height), and ``z`` a width-normalised relative depth (negative = closer to the
camera, MediaPipe convention). Do NOT feed pixel coordinates in here — the
thresholds below assume the [0, 1] space.

This module is intentionally decision-independent: it only derives stable
features and smooths/votes them. The temporal command decoder (which gesture
maps to which robot command) lives in ``gesture_classifier.py``.
"""

import math
from collections import deque

import numpy as np

# MediaPipe 21-landmark indices
WRIST = 0
THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP = 1, 2, 3, 4
INDEX_MCP, INDEX_PIP, INDEX_TIP = 5, 6, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_TIP = 9, 10, 12
RING_MCP, RING_PIP, RING_TIP = 13, 14, 16
PINKY_MCP, PINKY_PIP, PINKY_TIP = 17, 18, 20

TIPS = [INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP]
PIPS = [INDEX_PIP, MIDDLE_PIP, RING_PIP, PINKY_PIP]


# -- per-frame features ----------------------------------------------------

def _angle(a, b, c):
    """Interior angle at vertex ``b`` (degrees), using only x/y."""
    ba = a[:2] - b[:2]
    bc = c[:2] - b[:2]
    nba = np.linalg.norm(ba)
    nbc = np.linalg.norm(bc)
    if nba < 1e-6 or nbc < 1e-6:
        return 180.0
    cosang = np.clip(np.dot(ba, bc) / (nba * nbc), -1.0, 1.0)
    return math.degrees(math.acos(cosang))


def fingers_up(lms):
    """Return [thumb, index, middle, ring, pinky] where 1 == extended.

    Thumb uses the IP-joint angle plus a tip-vs-IP distance-from-wrist test;
    the other four use ``tip.y < pip.y`` (image-up). Preserving *which* finger
    is up (not just the count) is what lets POINTING be told apart from a noisy
    fist.
    """
    wrist = lms[WRIST]
    thumb_angle = _angle(lms[THUMB_MCP], lms[THUMB_IP], lms[THUMB_TIP])
    tip_d = np.linalg.norm(lms[THUMB_TIP][:2] - wrist[:2])
    ip_d = np.linalg.norm(lms[THUMB_IP][:2] - wrist[:2])
    thumb = 1 if (thumb_angle > 150.0 and tip_d - ip_d > 0.035) else 0

    out = [thumb]
    for tip, pip in zip(TIPS, PIPS):
        out.append(1 if lms[tip][1] < lms[pip][1] else 0)
    return out


_PATTERN_LABELS = {
    (0, 0, 0, 0, 0): 'FIST',
    (1, 1, 1, 1, 1): 'OPEN_PALM',
    (0, 1, 0, 0, 0): 'POINTING',
    (1, 0, 0, 0, 0): 'THUMBS_UP',
    (0, 1, 1, 0, 0): 'PEACE',
    (1, 1, 0, 0, 1): 'I_LOVE_YOU',
}


def label_from_finger_pattern(fingers):
    return _PATTERN_LABELS.get(tuple(fingers), 'CUSTOM')


def hand_openness(lms):
    """Continuous 0..1 openness, robust to half-curled hands.

    Ratio of mean fingertip-to-wrist distance against palm width, remapped so a
    flat open palm sits near 1.0 and a fist near 0.0.
    """
    palm_width = np.linalg.norm(lms[INDEX_MCP][:2] - lms[PINKY_MCP][:2])
    if palm_width < 1e-6:
        return 0.0
    wrist = lms[WRIST][:2]
    mean_tip = np.mean([np.linalg.norm(lms[t][:2] - wrist) for t in TIPS])
    ratio = mean_tip / palm_width
    return float(np.clip((ratio - 1.0) / 1.6, 0.0, 1.0))


def hand_center_and_span(lms):
    xs = lms[:, 0]
    ys = lms[:, 1]
    cx = float(xs.mean())
    cy = float(ys.mean())
    span = float(max(xs.max() - xs.min(), ys.max() - ys.min()))
    return cx, cy, span


def thumbs_down(lms, fingers):
    """Four fingers curled and the thumb pointing downward in the image."""
    if sum(fingers[1:]) != 0:
        return False
    return (lms[THUMB_TIP][1] - lms[THUMB_MCP][1]) > 0.08


def pointing_direction(lms):
    """Coarse pointing direction from index MCP -> index tip.

    Returns one of 'LEFT', 'RIGHT', 'UP', 'DOWN', 'TOWARDS_CAMERA', or a compass
    sector string. Only LEFT/RIGHT are used for movement commands.
    """
    mcp = lms[INDEX_MCP]
    tip = lms[INDEX_TIP]
    dx = tip[0] - mcp[0]
    dy = tip[1] - mcp[1]
    dz = mcp[2] - tip[2]
    plane_len = math.hypot(dx, dy)

    if dz > 0.12 and plane_len < 0.10:
        return 'TOWARDS_CAMERA'
    if abs(dx) > max(0.06, abs(dy) * 1.25):
        return 'RIGHT' if dx > 0 else 'LEFT'
    # fall back to up/down so callers can ignore non-horizontal points
    if abs(dy) > 0.06:
        return 'DOWN' if dy > 0 else 'UP'
    return 'NONE'


# -- stability filters -----------------------------------------------------

class LandmarkSmoother:
    """EMA smoother with per-frame jump rejection."""

    def __init__(self, alpha=0.55, jump_threshold=0.22):
        self.alpha = alpha
        self.jump_threshold = jump_threshold
        self._prev = None

    def reset(self):
        self._prev = None

    def update(self, lms):
        if lms is None:
            self._prev = None
            return None
        if self._prev is None:
            self._prev = lms.copy()
            return self._prev
        jump = np.linalg.norm(lms[:, :2] - self._prev[:, :2], axis=1).max()
        blend = 0.20 if jump > self.jump_threshold else self.alpha
        self._prev = blend * lms + (1.0 - blend) * self._prev
        return self._prev


class MajorityLabelFilter:
    """Switch to a new label only once it wins enough recent votes."""

    def __init__(self, history=7, votes=4):
        self.buf = deque(maxlen=history)
        self.votes = votes
        self.current = 'NONE'

    def reset(self):
        self.buf.clear()
        self.current = 'NONE'

    def update(self, label):
        self.buf.append(label)
        counts = {}
        for lab in self.buf:
            counts[lab] = counts.get(lab, 0) + 1
        best, n = max(counts.items(), key=lambda kv: kv[1])
        if n >= self.votes:
            self.current = best
        return self.current


def count_axis_reversals(values, min_sweep):
    """Count meaningful direction reversals in a 1-D sequence, ignoring jitter
    below ``min_sweep`` until the value has actually moved that far."""
    if len(values) < 3:
        return 0
    reversals = 0
    last_sign = 0
    anchor = values[0]
    for v in values[1:]:
        delta = v - anchor
        if abs(delta) < min_sweep:
            continue
        sign = 1 if delta > 0 else -1
        if last_sign and sign != last_sign:
            reversals += 1
        last_sign = sign
        anchor = v
    return reversals


class BeckonOscillationDetector:
    """Counts open<->closed 'curl' oscillations from the continuous openness
    signal (the reliable 'come closer' detector)."""

    def __init__(self, oscillations_required=2, window_sec=4.5,
                 open_threshold=0.62, closed_threshold=0.32,
                 min_half_period_sec=0.18):
        self.required = oscillations_required
        self.window = window_sec
        self.open_th = open_threshold
        self.closed_th = closed_threshold
        self.min_half = min_half_period_sec
        self.reset()

    def reset(self):
        self._state = None          # 'open' | 'closed'
        self._last_change = 0.0
        self._curls = deque()       # timestamps of closed->open completions

    def update(self, t, openness):
        new_state = self._state
        if openness >= self.open_th:
            new_state = 'open'
        elif openness <= self.closed_th:
            new_state = 'closed'

        if new_state != self._state:
            if (t - self._last_change) >= self.min_half:
                # Count the *closing* edge: a "curl" is the hand closing from an
                # open state, so a natural "curl twice" (open->close->open->close)
                # registers as 2. (Counting the re-open edge instead loses one.)
                if self._state == 'open' and new_state == 'closed':
                    self._curls.append(t)
                self._state = new_state
                self._last_change = t

        while self._curls and (t - self._curls[0]) > self.window:
            self._curls.popleft()
        return len(self._curls) >= self.required

    def consume(self):
        self._curls.clear()
