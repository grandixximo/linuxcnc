#!/bin/sh

set -eu #Needed so CI fails when anything is wrong
set -x

debian/configure
debian/update-dch-from-git
scripts/get-version-from-git | sed -re 's/^v(.*)$/\1/' >| VERSION; cat VERSION
git diff
apt-get --yes build-dep --indep-only .
DEB_BUILD_OPTIONS="parallel=$(nproc) nocheck"
export DEB_BUILD_OPTIONS
debuild -us -uc --build=source,all
