#!/usr/bin/env bash
# Lock the Logitech C930e. It forgets these settings after unplug or reboot.
# Run this script every time the camera is plugged back in.
set -euo pipefail

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "v4l2-ctl is missing. Install it with: sudo apt install -y v4l-utils" >&2
  exit 1
fi

# Capture node for C930e serial C88D48BE. video-index0 is the picture.
# video-index1 is a metadata node and will not produce frames.
DEV="${CAMERA_DEV:-/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e_C88D48BE-video-index0}"

if [[ ! -e "${DEV}" ]]; then
  echo "Camera not found at ${DEV}. Plug in the C930e and open the privacy shutter." >&2
  exit 1
fi

# One v4l2-ctl call sends every control in a single request. The C930e rejects
# the manual control while its automatic partner is still on, and reports that
# as "Permission denied". Turn each automatic mode off before setting the manual value.
# auto_exposure=1 is manual. exposure_dynamic_framerate=0 stops the camera from
# changing the frame rate on its own. Exposure 312 is the 30 fps shutter; gain 255
# brightens the LED-strip scene without dropping the frame rate. focus_absolute=15
# is the sharp peak. Values above about 40 go soft.
v4l2-ctl -d "${DEV}" -c focus_automatic_continuous=0
v4l2-ctl -d "${DEV}" -c focus_absolute=15
v4l2-ctl -d "${DEV}" -c auto_exposure=1
v4l2-ctl -d "${DEV}" -c exposure_time_absolute=312
v4l2-ctl -d "${DEV}" -c exposure_dynamic_framerate=0
v4l2-ctl -d "${DEV}" -c gain=255
v4l2-ctl -d "${DEV}" -c white_balance_automatic=0
v4l2-ctl -d "${DEV}" -c white_balance_temperature=4000

echo "Locked ${DEV}"
v4l2-ctl -d "${DEV}" \
  --get-ctrl focus_automatic_continuous,focus_absolute,auto_exposure,exposure_time_absolute,exposure_dynamic_framerate,gain,white_balance_automatic,white_balance_temperature
