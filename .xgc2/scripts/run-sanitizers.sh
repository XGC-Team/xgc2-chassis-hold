#!/usr/bin/env bash
# Runs the whole test suite under ThreadSanitizer, then under AddressSanitizer + UndefinedBehaviorSanitizer.
# Usage: run-sanitizers.sh <scratch directory>
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
work="${1:?scratch directory}"
mkdir -p "$work"
"$root/.xgc2/scripts/ensure-build-deps.sh"
for sanitizer in thread address,undefined; do
  build="$work/${sanitizer//,/-}"
  mkdir -p "$build"
  (
    cd "$build"
    cmake "$root" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXGC2_CHASSIS_HOLD_WERROR=ON \
      -DCMAKE_CXX_FLAGS="-fsanitize=$sanitizer -fno-sanitize-recover=undefined -fno-omit-frame-pointer"
    cmake --build . -- -j"${XGC2_BUILD_JOBS:-2}"
    ctest --output-on-failure
  )
done
