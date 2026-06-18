"""Depth-based hand segmentation and shape analysis for the QBot gesture node.

MediaPipe has no ARM64 wheels, so instead of RGB landmark detection we use the
Kinect *depth* image: the hand, raised toward the camera, is the nearest blob.
We segment it, then derive simple shape features (finger count, openness,
centroid, fingertip) that the gesture state machine turns into commands.

The Kinect's default libfreenect depth is 11-bit raw disparity: 0 and 2047 are
"no data", and lower values are *closer*. We segment adaptively from the nearest
valid percentile so we don't depend on a metric (mm) calibration that this
particular unit can't provide (its registration tables failed to load).
"""

from dataclasses import dataclass

import cv2
import numpy as np

NO_DATA = 2047  # libfreenect 11-bit "no return" sentinel


@dataclass
class HandFeatures:
    present: bool
    cx: float = 0.0          # centroid x, normalised [-1, 1] (left negative)
    cy: float = 0.0          # centroid y, normalised [-1, 1] (up negative)
    fingers: int = 0         # estimated extended-finger count (0..5)
    solidity: float = 0.0    # contour_area / hull_area (fist ~high, open ~low)
    area_frac: float = 0.0   # hand area as a fraction of the image
    tip_x: float = 0.0       # topmost contour point (fingertip), normalised
    tip_y: float = 0.0


def segment_hand(depth, near_band=60, min_area_frac=0.01, invert=False):
    """Return (mask, contour) for the nearest blob, or (None, None).

    depth: HxW uint16 (11-bit raw). near_band: width of the depth slab kept in
    front of the nearest surface. invert: set True if your unit reports higher =
    closer.
    """
    d = depth.astype(np.int32)
    valid = (d > 0) & (d < NO_DATA)
    if valid.sum() < 500:
        return None, None

    vals = d[valid]
    if invert:
        # higher = closer: keep the top slab
        near = np.percentile(vals, 98)
        slab = valid & (d >= near - near_band)
    else:
        # lower = closer (Kinect default): keep the bottom slab
        near = np.percentile(vals, 2)
        slab = valid & (d <= near + near_band)

    mask = (slab * 255).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area_frac * depth.size:
        return None, None
    return mask, c


def _count_fingers(contour, min_defect_depth=12.0):
    """Count extended fingers via convexity defects (valleys between fingers)."""
    if len(contour) < 5:
        return 0
    hull = cv2.convexHull(contour, returnPoints=False)
    if hull is None or len(hull) < 4:
        return 0
    defects = cv2.convexityDefects(contour, hull)
    if defects is None:
        return 0
    valleys = 0
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        depth_val = d / 256.0
        if depth_val < min_defect_depth:
            continue
        start = np.array(contour[s][0], dtype=float)
        end = np.array(contour[e][0], dtype=float)
        far = np.array(contour[f][0], dtype=float)
        a = np.linalg.norm(end - far)
        b = np.linalg.norm(start - far)
        ab = np.linalg.norm(start - end)
        if a * b == 0:
            continue
        # angle at the valley; finger gaps are sharp (< ~90 deg)
        cos_ang = (a * a + b * b - ab * ab) / (2 * a * b)
        cos_ang = max(-1.0, min(1.0, cos_ang))
        if np.degrees(np.arccos(cos_ang)) < 90:
            valleys += 1
    # N valleys between fingers => N+1 fingers, capped at 5
    return min(valleys + 1, 5) if valleys > 0 else 0


def analyse(depth, **seg_kwargs):
    """Full pipeline: depth image -> HandFeatures."""
    h, w = depth.shape[:2]
    mask, c = segment_hand(depth, **seg_kwargs)
    if c is None:
        return HandFeatures(present=False)

    area = cv2.contourArea(c)
    hull_pts = cv2.convexHull(c)
    hull_area = cv2.contourArea(hull_pts) if hull_pts is not None else area
    solidity = float(area / hull_area) if hull_area > 0 else 1.0

    M = cv2.moments(c)
    if M['m00'] == 0:
        return HandFeatures(present=False)
    cx_px = M['m10'] / M['m00']
    cy_px = M['m01'] / M['m00']

    # topmost point = likely fingertip when pointing
    tip = c[c[:, :, 1].argmin()][0]

    fingers = _count_fingers(c)
    # A nearly-solid blob with no valleys is a fist; a low-solidity one with a
    # single protrusion is a pointing finger.
    if fingers == 0 and solidity < 0.80:
        fingers = 1

    return HandFeatures(
        present=True,
        cx=(cx_px - w / 2.0) / (w / 2.0),
        cy=(cy_px - h / 2.0) / (h / 2.0),
        fingers=fingers,
        solidity=solidity,
        area_frac=float(area) / float(depth.size),
        tip_x=(float(tip[0]) - w / 2.0) / (w / 2.0),
        tip_y=(float(tip[1]) - h / 2.0) / (h / 2.0),
    )
