# Vendored hand-landmark model wrappers

`mp_palmdet.py` (MPPalmDet) and `mp_handpose.py` (MPHandPose) are vendored from
the **OpenCV Model Zoo** (https://github.com/opencv/opencv_zoo), Apache-2.0
licensed, and lightly modified: inference was switched from `cv2.dnn` to
`onnxruntime` (OpenCV 4.6 on Ubuntu 24.04 can't run the handpose ONNX graph).
All MediaPipe pre/post-processing (palm-anchor decode, rotated-ROI crop, 21-point
landmark decode) is unchanged.

The ONNX model weights are **not** committed — they are downloaded into
`../../models/` by `setup_qbot_env.sh` from:
- https://huggingface.co/opencv/palm_detection_mediapipe
- https://huggingface.co/opencv/handpose_estimation_mediapipe
