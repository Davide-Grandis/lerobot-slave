#!/usr/bin/env bash
set -euo pipefail

readonly CALIBRATOR=/opt/cf-robot/venv/bin/lerobot-calibrate

read -r -p 'Follower serial path (/dev/serial/by-id/...): ' FOLLOWER_PORT

if [[ ! -c "$FOLLOWER_PORT" ]]; then
  printf 'Not a serial character device: %s\n' "$FOLLOWER_PORT" >&2
  exit 1
fi

if ! sudo -u robot test -r "$FOLLOWER_PORT" ||
   ! sudo -u robot test -w "$FOLLOWER_PORT"; then
  printf 'The robot user cannot read and write this device: %s\n' "$FOLLOWER_PORT" >&2
  exit 1
fi

if [[ ! -x "$CALIBRATOR" ]]; then
  printf 'LeRobot calibration command not found or not executable: %s\n' "$CALIBRATOR" >&2
  exit 1
fi

exec sudo -u robot env HF_HOME=/var/lib/cf-robot "$CALIBRATOR" \
  --robot.type=so101_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.id=so101_follower
