#!/usr/bin/env bash
# Install LeRobot and OpenCV into the lerobot conda env. Run after: conda activate lerobot
set -euo pipefail

if [[ "${CONDA_DEFAULT_ENV:-}" != "lerobot" ]]; then
  echo "Activate the lerobot conda env first: conda activate lerobot" >&2
  exit 1
fi

LEROBOT_DIR="${HOME}/lerobot"
if [[ ! -d "${LEROBOT_DIR}/.git" ]]; then
  git clone https://github.com/huggingface/lerobot.git "${LEROBOT_DIR}"
else
  echo "LeRobot already cloned at ${LEROBOT_DIR}; leaving it unchanged."
fi

DESC="$(git -C "${LEROBOT_DIR}" describe --tags --always)"
echo "LeRobot version: ${DESC}"
echo "Expected revision is v0.6.2. This script does not pull or checkout a different revision."
if [[ "${DESC}" != v0.6.2 && "${DESC}" != v0.6.2-* ]]; then
  echo "WARNING: clone is ${DESC}, not v0.6.2." >&2
fi

pip install -e "${LEROBOT_DIR}[core_scripts,feetech,kinematics,training]"
pip uninstall -y opencv-python-headless
pip install "opencv-python>=4.9,<4.14"
pip install pytest
python -c "import cv2, lerobot, placo; print(cv2.__version__, cv2.aruco.ArucoDetector)"
