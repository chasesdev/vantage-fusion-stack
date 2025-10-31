#!/usr/bin/env bash
set -e
for dev in /dev/video*; do
  echo "=== $dev ==="
  v4l2-ctl -d "$dev" --all || true
  echo
done
