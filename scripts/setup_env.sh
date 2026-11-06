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

# There is no v0.6.2 git tag. The guide's "v0.6.2" is the package version on main
# (pyproject.toml). This script does not pull or checkout a different revision.
PY_VER="$(awk -F'"' '/^version = /{print $2; exit}' "${LEROBOT_DIR}/pyproject.toml")"
DESC="$(git -C "${LEROBOT_DIR}" describe --tags --always)"
echo "LeRobot package version: ${PY_VER} (${DESC})"
if [[ "${PY_VER}" != "0.6.2" ]]; then
  echo "WARNING: clone package version is ${PY_VER}, not 0.6.2." >&2
fi

pip install -e "${LEROBOT_DIR}[core_scripts,feetech,kinematics,training]"
pip uninstall -y opencv-python-headless
pip install "opencv-python>=4.9,<4.14"
pip install pytest
python -c "import cv2, lerobot, placo; print(cv2.__version__, cv2.aruco.ArucoDetector)"
