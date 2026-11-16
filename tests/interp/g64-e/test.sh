#!/bin/bash
# the motion control mode lines carry E; the three error programs stop at their error
for f in test negative ambiguous no-g64; do
    echo "== $f"
    rs274 -i test.ini -g $f.ngc 2>&1 | grep -E "MOTION_CONTROL|E word|angular tolerance" | sed 's/^ *[0-9]* N\.\.\.\.\. //'
done
