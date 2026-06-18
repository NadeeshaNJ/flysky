"""Hand tracking via ONNX MediaPipe models (palm detection + 21 landmarks).

MediaPipe itself has no ARM64 wheel, but its models converted to ONNX run fine
under onnxruntime (which does). We use OpenCV model-zoo's palm detector +
handpose estimator (vendored in ``mp_models/``) to get 21 reliable hand
landmarks from the Kinect RGB feed, then derive finger states + centroid as
``HandFeatures`` for the gesture classifier.

This replaced an earlier depth-silhouette approach: at arm's length the Kinect
depth blob couldn't separate an open palm from a fist, whereas the landmark model
gives per-finger joint positions (~100% detection, 0.98+ confidence in testing).
"""

import os
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import onnxruntime as ort

# onnxruntime 1.27 logs noisy GPU-discovery warnings on a headless Pi; quiet them.
ort.set_default_logger_severity(3)  # 3 = ERROR (hide INFO/WARNING)

from gesture_node.mp_models.mp_palmdet import MPPalmDet
from gesture_node.mp_models.mp_handpose import MPHandPose
from gesture_node import signals

# MediaPipe 21-landmark indices
WRIST = 0
TIPS = [8, 12, 16, 20]    # index, middle, ring, pinky fingertips
PIPS = [6, 10, 14, 18]    # corresponding PIP joints
THUMB_TIP, THUMB_MCP = 4, 2
PALM_IDS = [0, 5, 9, 13, 17]
INDEX_TIP = 8


@dataclass
class HandFeatures:
    present: bool
    cx: float = 0.0          # palm-centre x, normalised [-1, 1] (left negative)
    cy: float = 0.0          # palm-centre y, normalised [-1, 1] (up negative)
    fingers: int = 0         # extended-finger count (0..5)
    solidity: float = 0.0    # kept for API compat (unused with landmarks)
    area_frac: float = 0.0
    tip_x: float = 0.0       # index fingertip, normalised (for finger-circle)
    tip_y: float = 0.0
    index_only: bool = False  # pointing pose (only index extended)
    thumb_down: bool = False  # thumbs-down pose (fist + thumb pointing down)

    # Richer features for the temporal decoder (see signals.py). All in the
    # normalised [0, 1] image space — keep these separate from cx/cy/tip_x/tip_y
    # which are in the legacy [-1, 1] space.
    landmarks: Optional[np.ndarray] = None   # (21, 3) normalised
    finger_pattern: list = field(default_factory=list)  # [thumb,index,mid,ring,pinky]
    label: str = 'NONE'      # OPEN_PALM/POINTING/FIST/...
    openness: float = 0.0    # 0..1 continuous open-vs-fist
    nx: float = 0.0          # hand centre x in [0, 1]
    ny: float = 0.0          # hand centre y in [0, 1]
    span: float = 0.0        # bounding span in [0, 1]
    index_tip: tuple = (0.0, 0.0)        # index fingertip in [0, 1]
    pointing: str = 'NONE'   # LEFT/RIGHT/UP/DOWN/TOWARDS_CAMERA/NONE


class HandLandmarkTracker:
    def __init__(self, model_dir, score_threshold=0.5, conf_threshold=0.6):
        palm_path = os.path.join(model_dir, 'palm_detection_mediapipe_2023feb.onnx')
        hand_path = os.path.join(model_dir, 'handpose_estimation_mediapipe_2023feb.onnx')
        for p in (palm_path, hand_path):
            if not os.path.isfile(p):
                raise FileNotFoundError(
                    f'ONNX model not found: {p}. Run setup_qbot_env.sh to download it.')
        self.palm = MPPalmDet(palm_path, scoreThreshold=score_threshold)
        self.hand = MPHandPose(hand_path, confThreshold=conf_threshold)
        # Smooth landmarks before deriving features (reference order:
        # detect -> smooth -> features). Stateful across frames.
        self.smoother = signals.LandmarkSmoother()

    def process(self, bgr):
        """Run the pipeline on a BGR image; return HandFeatures."""
        h, w = bgr.shape[:2]
        dets = self.palm.infer(bgr)
        if dets is None or len(dets) == 0:
            self.smoother.reset()
            return HandFeatures(present=False)

        res = self.hand.infer(bgr, dets[0])
        if res is None:
            self.smoother.reset()
            return HandFeatures(present=False)

        lms_px = res[4:67].reshape(21, 3)            # screen-space (x, y, z) px
        wrist = lms_px[WRIST][:2]

        # Normalise to [0, 1]: x by width, y by height, z by width (MediaPipe-ish
        # relative depth). The temporal decoder consumes this; legacy [-1, 1]
        # fields below are kept so the current classifier still runs.
        norm = lms_px.copy().astype(np.float32)
        norm[:, 0] /= w
        norm[:, 1] /= h
        norm[:, 2] /= w
        norm = self.smoother.update(norm)

        def extended(tip, pip, slack=1.0):
            return np.linalg.norm(lms_px[tip][:2] - wrist) > np.linalg.norm(lms_px[pip][:2] - wrist) * slack

        ups = [extended(t, p) for t, p in zip(TIPS, PIPS)]
        thumb = bool(np.linalg.norm(lms_px[THUMB_TIP][:2] - wrist)
                     > np.linalg.norm(lms_px[THUMB_MCP][:2] - wrist) * 1.15)
        fingers = int(sum(ups) + thumb)
        index_only = bool(ups[0] and not ups[1] and not ups[2] and not ups[3])

        # Richer normalised features (signals.py).
        finger_pattern = signals.fingers_up(norm)
        label = signals.label_from_finger_pattern(finger_pattern)
        openness = signals.hand_openness(norm)
        nx, ny, span = signals.hand_center_and_span(norm)
        pointing = signals.pointing_direction(norm)

        # Thumbs-down: four fingers curled + thumb pointing downward. Derived from
        # the normalised, smoothed landmarks (signals.thumbs_down) so it stays
        # consistent with the finger pattern above. This is the *only* backward
        # cue now — a plain fist must not move the robot backward.
        thumb_down = signals.thumbs_down(norm, finger_pattern)

        palm_c = lms_px[PALM_IDS].mean(axis=0)
        tip = lms_px[INDEX_TIP]
        return HandFeatures(
            present=True,
            cx=(palm_c[0] - w / 2.0) / (w / 2.0),
            cy=(palm_c[1] - h / 2.0) / (h / 2.0),
            fingers=fingers,
            tip_x=(tip[0] - w / 2.0) / (w / 2.0),
            tip_y=(tip[1] - h / 2.0) / (h / 2.0),
            index_only=index_only,
            thumb_down=thumb_down,
            landmarks=norm,
            finger_pattern=finger_pattern,
            label=label,
            openness=openness,
            nx=nx, ny=ny, span=span,
            index_tip=(float(norm[INDEX_TIP][0]), float(norm[INDEX_TIP][1])),
            pointing=pointing,
        )
