#!/usr/bin/env bash
DEV=${1:-/dev/video2}
set -x
v4l2-ctl -d "$DEV" --set-fmt-video=width=3840,height=2160,pixelformat=NV12
v4l2-ctl -d "$DEV" --set-parm=60
v4l2-ctl -d "$DEV" --stream-mmap=4 --stream-count=120 --stream-to=/dev/null
