#!/bin/bash
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
cd "$HERE"
source $REPO/scripts/rip-environment >/dev/null
unset DISPLAY
rm -f /tmp/linuxcnc.lock frames.bin
export LINUXCNC_WS_CAPTURE=$PWD/frames.bin LINUXCNC_WS_CAPTURE_MAX=50000
timeout 420 linuxcnc -r bench.ini > run.log 2>&1
echo exit=$?
