#!/usr/bin/env bash
# Builds the udp component of the pinned XRPC checkout and installs it into a prefix: what CI uses until the
# release train publishes an XRPC package that carries it. The pin is .xgc2/xrpc.ref.
# Usage: install-xrpc-udp.sh <xrpc checkout> <prefix> [further cmake arguments]
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source="${1:?XRPC checkout}"
prefix="${2:?install prefix}"
shift 2
ref="$(tr -d '[:space:]' < "$root/.xgc2/xrpc.ref")"
[[ "$ref" =~ ^[0-9a-f]{40}$ ]]
test "$(git -C "$source" rev-parse HEAD)" = "$ref"

# XRPC needs CMake 3.16; Ubuntu 18.04 ships 3.10. The PyPI package is a plain binary and works as root in the image.
cmake_version="$(cmake --version | sed -n '1s/^[^0-9]*//p')"
if [[ "$(printf '3.16\n%s\n' "$cmake_version" | sort -V | head -n 1)" != 3.16 ]]; then
  python3 -m pip install --upgrade pip
  python3 -m pip install 'cmake>=3.16,<4'
  hash -r
fi

build="$prefix.build"
mkdir -p "$build"
(
  cd "$build"
  cmake "$source" -DCMAKE_BUILD_TYPE=Release -DXGC2_XRPC_COMPONENTS=udp -DBUILD_TESTING=OFF \
    "-DCMAKE_INSTALL_PREFIX=$prefix" "$@"
  cmake --build . -- -j"${XGC2_BUILD_JOBS:-2}"
  cmake --build . --target install
)
