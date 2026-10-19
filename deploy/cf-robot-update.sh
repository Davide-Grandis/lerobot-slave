#!/usr/bin/env bash
set -euo pipefail

readonly REPO=/opt/cf-robot
readonly REQUEST=/var/lib/cf-robot/update-request
readonly STATUS=/var/lib/cf-robot/update-status.json
readonly FOLLOWER_SERVICE=cf-robot-follower

request_id=""
old_commit=""
stage="read-request"
service_was_active=0

write_status() {
  local state="$1"
  local revision="$2"
  local temporary="${STATUS}.tmp"
  printf '{"request_id":"%s","state":"%s","stage":"%s","revision":"%s"}\n' \
    "$request_id" "$state" "$stage" "$revision" > "$temporary"
  chmod 0644 "$temporary"
  mv -f "$temporary" "$STATUS"
}

finish_on_error() {
  local result=$?
  if (( result != 0 )); then
    write_status "failed" "$old_commit" || true
    if (( service_was_active )); then
      systemctl start "$FOLLOWER_SERVICE" || true
    fi
  fi
}

if [[ ! -f "$REQUEST" ]]; then
  exit 0
fi
request_id="$(cat "$REQUEST")"
rm -f "$REQUEST"
if [[ ! "$request_id" =~ ^[[:xdigit:]]{32}$ ]]; then
  echo "Invalid update request marker" >&2
  exit 1
fi

old_commit="$(git -C "$REPO" rev-parse HEAD)"
write_status "running" "$old_commit"
trap finish_on_error EXIT

stage="validate-checkout"
origin="$(git -C "$REPO" remote get-url origin)"
case "$origin" in
  https://github.com/Davide-Grandis/lerobot-slave.git|git@github.com:Davide-Grandis/lerobot-slave.git|ssh://git@github.com/Davide-Grandis/lerobot-slave.git)
    ;;
  *)
    echo "Refusing update from unrecognized origin: $origin" >&2
    exit 1
    ;;
esac

if [[ "$(git -C "$REPO" branch --show-current)" != "main" ]]; then
  echo "Refusing update: checkout is not on main" >&2
  exit 1
fi
if [[ -n "$(git -C "$REPO" status --porcelain)" ]]; then
  echo "Refusing update: checkout has local changes" >&2
  exit 1
fi
if ! systemctl is-active --quiet "$FOLLOWER_SERVICE"; then
  echo "Refusing update: follower service is not active" >&2
  exit 1
fi
service_was_active=1

stage="fetch"
git -C "$REPO" fetch origin main
remote_commit="$(git -C "$REPO" rev-parse FETCH_HEAD)"
if [[ "$remote_commit" == "$old_commit" ]]; then
  stage="restart-follower"
  systemctl restart "$FOLLOWER_SERVICE"
  write_status "succeeded" "$old_commit"
  trap - EXIT
  exit 0
fi

stage="stop-follower"
systemctl stop "$FOLLOWER_SERVICE"
stage="fast-forward"
git -C "$REPO" merge --ff-only FETCH_HEAD

stage="refresh-systemd"
install -m 0644 "$REPO/deploy/cf-robot-follower.service" \
  /etc/systemd/system/cf-robot-follower.service
install -m 0644 "$REPO/deploy/cf-robot-update.service" \
  /etc/systemd/system/cf-robot-update.service
install -m 0644 "$REPO/deploy/cf-robot-update.path" \
  /etc/systemd/system/cf-robot-update.path
install -m 0755 "$REPO/deploy/cf-robot-update.sh" \
  /usr/local/sbin/cf-robot-update
systemctl daemon-reload
systemctl restart cf-robot-update.path

new_commit="$(git -C "$REPO" rev-parse HEAD)"
if ! git -C "$REPO" diff --quiet "$old_commit" "$new_commit" -- pyproject.toml; then
  stage="install-dependencies"
  TMPDIR=/var/tmp /opt/cf-robot/venv/bin/python -m pip install --no-cache-dir \
    -e "$REPO" 'lerobot[feetech]==0.6.1'
fi
/opt/cf-robot/venv/bin/python -m pip check

stage="start-follower"
systemctl start "$FOLLOWER_SERVICE"
write_status "succeeded" "$new_commit"
trap - EXIT
