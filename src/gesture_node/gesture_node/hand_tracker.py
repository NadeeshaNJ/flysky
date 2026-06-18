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
from dataclasses import dataclass

import numpy as np

from gesture_node.mp_models.mp_palmdet import MPPalmDet
from gesture_node.mp_models.mp_handpose import MPHandPose

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

    def process(self, bgr):
        """Run the pipeline on a BGR image; return HandFeatures."""
        h, w = bgr.shape[:2]
        dets = self.palm.infer(bgr)
        if dets is None or len(dets) == 0:
            return HandFeatures(present=False)

        res = self.hand.infer(bgr, dets[0])
        if res is None:
            return HandFeatures(present=False)

        lms = res[4:67].reshape(21, 3)[:, :2]   # screen-space (x, y) px
        wrist = lms[WRIST]

        def extended(tip, pip, slack=1.0):
            return np.linalg.norm(lms[tip] - wrist) > np.linalg.norm(lms[pip] - wrist) * slack

        ups = [extended(t, p) for t, p in zip(TIPS, PIPS)]
        thumb = bool(np.linalg.norm(lms[THUMB_TIP] - wrist)
                     > np.linalg.norm(lms[THUMB_MCP] - wrist) * 1.15)
        fingers = int(sum(ups) + thumb)
        index_only = bool(ups[0] and not ups[1] and not ups[2] and not ups[3])

        palm_c = lms[PALM_IDS].mean(axis=0)
        tip = lms[INDEX_TIP]
        return HandFeatures(
            present=True,
            cx=(palm_c[0] - w / 2.0) / (w / 2.0),
            cy=(palm_c[1] - h / 2.0) / (h / 2.0),
            fingers=fingers,
            tip_x=(tip[0] - w / 2.0) / (w / 2.0),
            tip_y=(tip[1] - h / 2.0) / (h / 2.0),
            index_only=index_only,
        )
