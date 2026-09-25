#!/bin/sh
# install.sh - install so101d as a systemd service.
#   cd daemon && sudo ./systemd/install.sh
# Idempotent: re-run it after every "git pull" to update the binaries.
set -e
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
cd "$(dirname "$0")/.."                       # daemon/
OWNER=${SUDO_USER:-root}
HOME_DIR=$(getent passwd "$OWNER" | cut -d: -f6)
CAL=$HOME_DIR/.cache/huggingface/lerobot/calibration

echo "== build (as $OWNER)"
sudo -u "$OWNER" make -C ../driver >/dev/null
sudo -u "$OWNER" make >/dev/null

echo "== user and group so101"
getent group so101 >/dev/null || groupadd --system so101
id so101 >/dev/null 2>&1 || useradd --system --gid so101 --groups dialout \
    --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin so101
[ "$OWNER" != root ] && usermod -aG so101 "$OWNER"

echo "== binaries"
systemctl is-active --quiet so101d && systemctl stop so101d || true
install -m 755 build/so101d build/so101ctl build/so101_log /usr/local/bin/
install -D -m 755 systemd/so101-wait-ports /usr/local/libexec/so101-wait-ports

echo "== calibration -> /etc/so101"
install -d -m 755 /etc/so101
install -m 644 "$CAL/teleoperators/so_leader/so101_leader.json" /etc/so101/
install -m 644 "$CAL/robots/so_follower/so101_follower.json" /etc/so101/

echo "== configuration and unit"
[ -f /etc/default/so101d ] || install -m 644 systemd/so101d.default /etc/default/so101d
install -m 644 systemd/so101d.service /etc/systemd/system/so101d.service
# leftovers from manual (sudo) runs would be owned by root
rm -f /dev/shm/so101 /dev/shm/sem.so101_cmd_lock /dev/shm/sem.so101_cmd_ready
systemctl daemon-reload

cat <<MSG

Installed. Next:
  sudo systemctl start so101d        # start now
  sudo systemctl enable so101d       # start on boot
  journalctl -u so101d -f            # logs
  so101ctl watch                     # (log out/in once so '$OWNER' is in group so101)
Config: /etc/default/so101d   Calibration: /etc/so101/
MSG
