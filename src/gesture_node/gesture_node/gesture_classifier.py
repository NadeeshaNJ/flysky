"""Temporal gesture decoder: a stream of HandFeatures -> command events.

Ported from the laptop MediaPipe project (see GESTURE_ALGORITHM_REFERENCE.md).
Unlike a "run recogniser, publish label" approach, commands are confirmed from
the *temporal* evolution of robust per-frame features (continuous hand openness,
normalised landmark geometry, majority-voted pose label) supplied on
``HandFeatures`` by ``hand_tracker.py`` / ``signals.py``.

Gesture vocabulary (decided with the team, 2026-06-18):

  stop        open palm held still                  -> immediate halt
  forward     "come closer": curl the hand twice    -> drive forward
              (openness open->closed->open x2)
  backward    thumbs-down held ~0.6s                -> drive backward
  rotate360   index finger drawing a circle         -> spin 360 in place
  turn_left   index finger pointed & held LEFT      -> sidestep-left maneuver
  turn_right  index finger pointed & held RIGHT     -> sidestep-right maneuver
  tail_wag    open-hand side-to-side wave (>=3 rev) -> wag 3x

All thresholds are exposed for on-hand calibration; the node logs features at
debug level. Coordinates here are the normalised [0, 1] image space carried on
HandFeatures (nx/ny/openness/index_tip/span/label/pointing) — never mix in the
legacy [-1, 1] cx/cy fields.
"""

import math

from gesture_node.signals import (
    MajorityLabelFilter,
    BeckonOscillationDetector,
    count_axis_reversals,
)


class GestureClassifier:
    def __init__(self,
                 cooldown=1.5,            # s global lockout after a fire
                 span_min=0.055,          # reject tiny/noisy detections
                 # general
                 absence_grace=0.3,       # s of lost detection tolerated before reset
                 # stop
                 stop_open=0.70,          # openness for an "open palm"
                 stop_hold=0.45,          # s the palm must persist
                 stop_motion_tol=0.04,    # max palm drift while "still"
                 wave_open=0.55,          # openness to keep collecting the wave/palm track
                 # forward (beckon)
                 beckon_oscillations=2,
                 beckon_window=4.5,
                 beckon_open=0.62,
                 beckon_closed=0.32,
                 beckon_min_half=0.18,
                 # backward (thumbs-down held) — a plain fist must NOT trigger
                 # this; only an explicit thumbs-down does (it clashed with the
                 # 'forward' beckon, which closes the hand into fists).
                 back_hold=0.6,           # s the thumbs-down pose must persist
                 # rotate360 (index circle)
                 rotate_window=3.2,
                 rotate_min_samples=12,
                 rotate_min_duration=0.55,
                 rotate_min_radius=0.04,
                 rotate_net_angle=1.55 * math.pi,
                 rotate_total_angle=1.75 * math.pi,
                 rotate_quadrants=4,
                 # turn (index point held left/right)
                 point_hold=0.55,
                 mirror_horizontal=True,  # camera image is mirrored vs the user
                 # tail_wag (open-hand wave)
                 wag_window=2.4,
                 wag_min_samples=8,
                 wag_amp=0.08,
                 wag_sweep=0.18,
                 wag_reversals=3):
        self.cooldown = cooldown
        self.span_min = span_min
        self.absence_grace = absence_grace

        self.stop_open = stop_open
        self.stop_hold = stop_hold
        self.stop_motion_tol = stop_motion_tol
        self.wave_open = wave_open

        self.back_hold = back_hold

        self.rotate_window = rotate_window
        self.rotate_min_samples = rotate_min_samples
        self.rotate_min_duration = rotate_min_duration
        self.rotate_min_radius = rotate_min_radius
        self.rotate_net_angle = rotate_net_angle
        self.rotate_total_angle = rotate_total_angle
        self.rotate_quadrants = rotate_quadrants

        self.point_hold = point_hold
        self.mirror_horizontal = mirror_horizontal

        self.wag_window = wag_window
        self.wag_min_samples = wag_min_samples
        self.wag_amp = wag_amp
        self.wag_sweep = wag_sweep
        self.wag_reversals = wag_reversals

        self.labels = MajorityLabelFilter()
        self.beckon = BeckonOscillationDetector(
            beckon_oscillations, beckon_window, beckon_open, beckon_closed,
            beckon_min_half)

        # sliding tracks of (t, value...) — pruned per detector window
        self.palm_track = []     # (t, cx, cy)  while open
        self.point_track = []    # (t, tip_x, tip_y)  while POINTING
        self._open_since = None
        self._thumbdown_since = None
        self._point_dir = None
        self._point_since = None
        self._last_present = -1e9
        self._locked_until = 0.0
        self._t_cur = 0.0

    def reset(self):
        self.labels.reset()
        self.beckon.reset()
        self.palm_track.clear()
        self.point_track.clear()
        self._open_since = None
        self._thumbdown_since = None
        self._point_dir = None
        self._point_since = None

    # -- main entry --------------------------------------------------------
    def update(self, t, feat):
        """Feed one frame; return a command string or None."""
        self._t_cur = t
        if not feat.present or feat.span < self.span_min:
            # Tolerate brief detection dropouts (common under wave/curl motion
            # blur) — only wipe the buffers if the hand is gone long enough that
            # an in-progress gesture is genuinely over.
            if (t - self._last_present) > self.absence_grace:
                self.reset()
            return None
        self._last_present = t

        label = self.labels.update(feat.label)
        openness = feat.openness
        cx, cy = feat.nx, feat.ny
        tip = feat.index_tip

        # feed every-frame detectors
        beckon_ready = self.beckon.update(t, openness)

        # Palm track (used by BOTH stop and tail_wag). Collect while the hand is
        # reasonably open (wave_open, 0.55) and only clear when it clearly isn't —
        # a waving hand tilts/blurs and dips under the 0.70 stop threshold, so
        # gating the buffer at 0.70 would wipe the wave before it accumulates.
        if openness >= self.wave_open:
            self.palm_track.append((t, cx, cy))
        else:
            self.palm_track.clear()
        self._prune(self.palm_track, t, self.wag_window)

        # Stop needs a *clearly* open palm (0.70) held — track that separately.
        if openness >= self.stop_open:
            if self._open_since is None:
                self._open_since = t
        else:
            self._open_since = None

        if label == 'POINTING':
            self.point_track.append((t, tip[0], tip[1]))
        else:
            self.point_track.clear()
        self._prune(self.point_track, t, self.rotate_window)

        # thumbs-down hold (backward) — the ONLY backward cue. A plain closed fist
        # no longer triggers backward: the 'forward' beckon closes the hand into
        # fists, so a held fist used to clash with it. An explicit thumbs-down
        # (four fingers curled + thumb pointing down) is unambiguous.
        if feat.thumb_down:
            if self._thumbdown_since is None:
                self._thumbdown_since = t
        else:
            self._thumbdown_since = None

        # point hold (turn) — same horizontal direction held
        if label == 'POINTING' and feat.pointing in ('LEFT', 'RIGHT'):
            if feat.pointing != self._point_dir:
                self._point_dir = feat.pointing
                self._point_since = t
        else:
            self._point_dir = None
            self._point_since = None

        # STOP is always responsive: it must abort a running maneuver even
        # during the post-gesture cooldown.
        if self._stop_ready(t):
            self._fire()
            return 'stop'

        if t < self._locked_until:
            return None

        # Evaluation order mirrors the reference decoder.
        if self._wave_ready(t):
            self._fire()
            return 'tail_wag'

        if beckon_ready:
            self.beckon.consume()
            self._fire()
            return 'forward'

        if self._circle_ready(t):
            self._fire()
            return 'rotate360'

        turn = self._turn_ready(t)
        if turn:
            self._fire()
            return turn

        if self._thumbdown_since is not None and (t - self._thumbdown_since) >= self.back_hold:
            self._fire()
            return 'backward'

        return None

    # -- detectors ---------------------------------------------------------
    def _stop_ready(self, t):
        if self._open_since is None or (t - self._open_since) < self.stop_hold:
            return False
        recent = [p for p in self.palm_track if t - p[0] <= 0.5]
        if len(recent) < 3:
            return False
        xs = [p[1] for p in recent]
        # if the palm is sweeping sideways it's probably a wave, not a stop
        return (max(xs) - min(xs)) <= self.stop_motion_tol * 3

    def _wave_ready(self, t):
        pts = [p for p in self.palm_track if t - p[0] <= self.wag_window]
        if len(pts) < self.wag_min_samples:
            return False
        xs = [p[1] for p in pts]
        if (max(xs) - min(xs)) < self.wag_amp * 2:
            return False
        rev = count_axis_reversals(xs, self.wag_sweep / 2.4)
        return rev >= self.wag_reversals

    def _circle_ready(self, t):
        pts = [p for p in self.point_track if t - p[0] <= self.rotate_window]
        if len(pts) < self.rotate_min_samples:
            return False
        if (pts[-1][0] - pts[0][0]) < self.rotate_min_duration:
            return False

        xs = [p[1] for p in pts]
        ys = [p[2] for p in pts]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        radii = [math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)]
        mean_r = sum(radii) / len(radii)
        if mean_r < self.rotate_min_radius:
            return False
        var = sum((r - mean_r) ** 2 for r in radii) / len(radii)
        if math.sqrt(var) > mean_r * 0.95:
            return False

        angs = [math.atan2(y - cy, x - cx) for x, y in zip(xs, ys)]
        net = 0.0
        total = 0.0
        for a, b in zip(angs, angs[1:]):
            d = b - a
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            net += d
            total += abs(d)
        if abs(net) < self.rotate_net_angle or total < self.rotate_total_angle:
            return False

        quads = set()
        for x, y in zip(xs, ys):
            quads.add((1 if x >= cx else 0, 1 if y >= cy else 0))
        return len(quads) >= self.rotate_quadrants

    def _turn_ready(self, t):
        if self._point_since is None:
            return None
        if (t - self._point_since) < self.point_hold:
            return None
        d = self._point_dir
        if self.mirror_horizontal:
            # camera mirrors the user: pointing image-RIGHT means the user's left
            return 'turn_left' if d == 'RIGHT' else 'turn_right'
        return 'turn_left' if d == 'LEFT' else 'turn_right'

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def _prune(track, t, window):
        while track and (t - track[0][0]) > window:
            track.pop(0)

    def _fire(self):
        # cooldown runs from the current frame's time, stashed by update().
        self._locked_until = self._t_cur + self.cooldown
        self.palm_track.clear()
        self.point_track.clear()
        self._open_since = None
        self._thumbdown_since = None
        self._point_dir = None
        self._point_since = None
        self.labels.reset()
