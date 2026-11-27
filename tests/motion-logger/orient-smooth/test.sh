#!/bin/bash -e

# orient.ngc with the tilt axis on A, then the same program with it on V
rm -f out.motion-logger out-v.motion-logger orient-v.ngc test-v.ini mock-motion-v.hal

linuxcnc -r test.ini

sed 's/\bA\([-0-9.]\)/V\1/g' orient.ngc > orient-v.ngc
sed -e 's/^COORDINATES = X Y Z A C$/COORDINATES = X Y Z V C/' \
    -e 's/coordinates=XYZAC$/coordinates=XYZVC/' \
    -e 's/^\[AXIS_A\]$/[AXIS_V]\nTYPE = ANGULAR/' \
    -e 's/^HALFILE = mock-motion.hal$/HALFILE = mock-motion-v.hal/' test.ini > test-v.ini
sed 's/out.motion-logger/out-v.motion-logger/' mock-motion.hal > mock-motion-v.hal
for p in '^COORDINATES = X Y Z V C$' 'coordinates=XYZVC$' '^HALFILE = mock-motion-v.hal$'; do
    grep -q "$p" test-v.ini
done
grep -A1 '^\[AXIS_V\]$' test-v.ini | grep -q '^TYPE = ANGULAR$'

ORIENT_NGC=orient-v.ngc linuxcnc -r test-v.ini
