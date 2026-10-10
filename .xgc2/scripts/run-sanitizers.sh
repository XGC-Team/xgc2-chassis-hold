#!/usr/bin/env bash
# Runs the whole test suite under ThreadSanitizer, then under AddressSanitizer + UndefinedBehaviorSanitizer; the XRPC
# udp component is built with the same sanitizer.
# Usage: run-sanitizers.sh <scratch directory> <dependency checkouts directory>   (the directory holds xrpc/)
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
work="${1:?scratch directory}"
deps="${2:?dependency checkouts}"
mkdir -p "$work"
"$root/.xgc2/scripts/ensure-build-deps.sh"
for sanitizer in thread address,undefined; do
  name="${sanitizer//,/-}"
  flags="-fsanitize=$sanitizer -fno-sanitize-recover=undefined -fno-omit-frame-pointer"
  prefix="$work/xrpc-$name"
  "$root/.xgc2/scripts/install-xrpc-udp.sh" "$deps/xrpc" "$prefix" "-DCMAKE_CXX_FLAGS=$flags"
  build="$work/$name"
  mkdir -p "$build"
  (
    cd "$build"
    cmake "$root" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXGC2_CHASSIS_HOLD_WERROR=ON \
      "-DCMAKE_PREFIX_PATH=$prefix" "-DCMAKE_CXX_FLAGS=$flags"
    cmake --build . -- -j"${XGC2_BUILD_JOBS:-2}"
    LD_LIBRARY_PATH="$prefix/lib" ctest --output-on-failure
  )
done
