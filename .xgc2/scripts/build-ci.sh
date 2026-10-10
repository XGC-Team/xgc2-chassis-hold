#!/usr/bin/env bash
# CI entry point inside a build container: packages the product for PACKAGE_DISTRIBUTION, installs the packages
# and runs the tests again against the installed result.
# Usage: build-ci.sh <scratch directory>
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
work="${1:?scratch directory}"
mkdir -p "$work"
"$root/.xgc2/scripts/ensure-build-deps.sh"
"$root/.xgc2/scripts/build_debs.sh" "$work/hold" "$root/.ci/debs"
dpkg -i "$root/.ci/debs/"*.deb
mkdir -p "$work/installed"
cd "$work/installed"
cmake "$root/test" -DCMAKE_BUILD_TYPE=Debug
cmake --build . -- -j"${XGC2_BUILD_JOBS:-2}"
ctest --output-on-failure
