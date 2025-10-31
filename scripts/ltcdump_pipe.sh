#!/usr/bin/env bash
DEV=${LTC_AUDIO_DEVICE:-hw:0,0}
arecord -f S16_LE -r 48000 -c 1 -D "$DEV" -q | ltcdump -
