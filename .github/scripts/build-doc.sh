#!/bin/sh

set -eu #Needed so CI fails when anything is wrong
set -x

cd src
./autogen.sh
# Canary: run configure with explicit =pdf so any regression in
# asciidoctor-pdf / fontTools / NotoSerifCJK on the runner image
# fails fast (hard error from configure.ac).  No build, just probe.
./configure --disable-check-runtime-deps --enable-build-documentation=pdf
./configure --disable-check-runtime-deps \
            --enable-build-documentation=html \
            --enable-build-documentation-translation
make -O -j$((1+$(nproc))) manpages
make -O -j$((1+$(nproc))) translateddocs
make -O -j$((1+$(nproc))) docs
# Note that the package build covers html docs
