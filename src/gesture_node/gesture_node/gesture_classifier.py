"""Temporal gesture classifier: a stream of HandFeatures -> command events.

Gesture vocabulary (see README / CLAUDE.md). Static poses and dynamic motions
are distinguished over a short sliding window:

  stop        open palm (>=4 fingers) held still           -> immediate halt
  forward     "come closer": curl the hand twice            -> drive forward
              (open->fist alternation, >=2 times)
  backward    closed fist held still ~1s                     -> drive backward
  rotate360   one finger drawing a circle                    -> spin 360 in place
  turn_left   open-hand swipe toward image-left              -> sidestep-left maneuver
  turn_right  open-hand swipe toward image-right             -> sidestep-right maneuver
  tail_wag    rapid side-to-side hand wave (>=3 reversals)   -> wag 3x

Thresholds are deliberately exposed; they need on-hand calibration (we can't
tune them blind). The node logs features at debug level to help.
"""

import math
from collections import deque


class GestureClassifier:
    def __init__(self,
                 window=2.5,           # s of history kept
                 cooldown=1.5,         # s lockout after firing
                 stop_hold=0.3,        # s open palm must persist
                 back_hold=1.0,        # s fist must persist
                 swipe_window=0.7,     # s to measure a swipe
                 swipe_dx=0.5,         # min net horizontal travel (normalised)
                 wag_window=1.0,       # s to measure a wag
                 wag_reversals=3,      # direction changes for a wag
                 rotate_window=2.0,    # s to accumulate finger circle
                 rotate_angle=4.7,     # rad (~1.5 turns) to trigger rotate
                 motion_deadband=0.04):
        self.window = window
        self.cooldown = cooldown
        self.stop_hold = stop_hold
        self.back_hold = back_hold
        self.swipe_window = swipe_window
        self.swipe_dx = swipe_dx
        self.wag_window = wag_window
        self.wag_reversals = wag_reversals
        self.rotate_window = rotate_window
        self.rotate_angle = rotate_angle
        self.deadband = motion_deadband

        self.buf = deque()          # (t, feat)
        self._locked_until = 0.0

    def update(self, t, feat):
        """Feed one frame; return a command string or None."""
        self.buf.append((t, feat))
        while self.buf and t - self.buf[0][0] > self.window:
            self.buf.popleft()

        if t < self._locked_until:
            return None

        cmd = self._classify(t)
        if cmd:
            self._locked_until = t + self.cooldown
            self.buf.clear()
        return cmd

    # -- helpers ----------------------------------------------------------
    def _recent(self, t, span):
        return [(tt, f) for (tt, f) in self.buf if t - tt <= span]

    def _present(self, items):
        return [f for _, f in items if f.present]

    def _classify(self, t):
        present = self._present(self.buf)
        if not present:
            return None

        # 1) STOP — open palm *held* (full stop_hold), low motion (top priority).
        held = self._recent(t, self.stop_hold)
        if self._sustained(t, self.stop_hold, lambda f: f.fingers >= 4) and \
                self._span_motion(held) < self.deadband * 2:
            return 'stop'

        # 2) FORWARD — curl twice (>=2 open->closed transitions).
        if self._curl_count() >= 2:
            return 'forward'

        # 3) ROTATE360 — single finger drawing a circle.
        if self._finger_circle(t) >= self.rotate_angle:
            return 'rotate360'

        # 4) TAIL_WAG — rapid side-to-side (many reversals, small net).
        wag = self._recent(t, self.wag_window)
        rev, net = self._reversals_and_net(wag)
        if rev >= self.wag_reversals and abs(net) < self.swipe_dx:
            return 'tail_wag'

        # 5) TURN — directional open-hand swipe.
        swipe = self._recent(t, self.swipe_window)
        sp = self._present(swipe)
        if len(sp) >= 3 and sum(f.fingers >= 3 for f in sp) >= len(sp) // 2:
            _, net_s = self._reversals_and_net(swipe)
            if net_s <= -self.swipe_dx:
                return 'turn_left'
            if net_s >= self.swipe_dx:
                return 'turn_right'

        # 6) BACKWARD — fist *held still* for back_hold (lowest priority).
        back = self._recent(t, self.back_hold)
        if self._sustained(t, self.back_hold,
                           lambda f: f.fingers <= 1 and f.solidity >= 0.80) and \
                self._span_motion(back) < self.deadband * 2 and self._curl_count() == 0:
            return 'backward'

        return None

    def _sustained(self, t, span, pred, min_frames=3):
        """True if the buffer covers >= span seconds and every present frame in
        the last `span` satisfies `pred` (i.e. the pose was actually held)."""
        items = self._recent(t, span)
        if not items or (t - items[0][0]) < span * 0.9:
            return False
        pres = self._present(items)
        return len(pres) >= min_frames and all(pred(f) for f in pres)

    def _curl_count(self):
        """Count open->closed transitions across the whole buffer."""
        states = []
        for _, f in self.buf:
            if not f.present:
                continue
            if f.fingers >= 4:
                states.append('open')
            elif f.fingers <= 1:
                states.append('closed')
        transitions = 0
        for a, b in zip(states, states[1:]):
            if a == 'open' and b == 'closed':
                transitions += 1
        return transitions

    def _span_motion(self, items):
        pts = [(f.cx, f.cy) for _, f in items if f.present]
        if len(pts) < 2:
            return 0.0
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def _reversals_and_net(self, items):
        xs = [f.cx for _, f in items if f.present]
        if len(xs) < 3:
            return 0, 0.0
        reversals, last_sign = 0, 0
        for a, b in zip(xs, xs[1:]):
            dx = b - a
            if abs(dx) < self.deadband:
                continue
            sign = 1 if dx > 0 else -1
            if last_sign and sign != last_sign:
                reversals += 1
            last_sign = sign
        return reversals, xs[-1] - xs[0]

    def _finger_circle(self, t):
        items = self._recent(t, self.rotate_window)
        angs = []
        for _, f in items:
            if f.present and f.fingers == 1:
                angs.append(math.atan2(f.tip_y - f.cy, f.tip_x - f.cx))
        if len(angs) < 5:
            return 0.0
        total = 0.0
        for a, b in zip(angs, angs[1:]):
            d = b - a
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            total += d
        return abs(total)
