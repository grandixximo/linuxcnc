#!/bin/bash
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
cd "$HERE"
source $REPO/scripts/rip-environment >/dev/null
unset DISPLAY
rm -f /tmp/linuxcnc.lock
export LINUXCNC_WS_TIMING=1 LINUXCNC_WS_TIMING_DUMP=$PWD/tdump.txt
timeout 120 linuxcnc -r bench.ini > timing.log 2>&1
echo exit=$?
