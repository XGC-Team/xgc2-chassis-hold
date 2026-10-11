#!/usr/bin/env bash
# Builds the udp component of the pinned XRPC checkout, and the http component on every distribution but bionic (the
# Boost it needs is newer than bionic's), and installs them into a prefix: what CI uses until the release train publishes
# the XRPC packages. The pin is .xgc2/xrpc.ref.
# Usage: install-xrpc.sh <xrpc checkout> <prefix> [further cmake arguments]
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source="${1:?XRPC checkout}"
prefix="${2:?install prefix}"
shift 2
ref="$(tr -d '[:space:]' < "$root/.xgc2/xrpc.ref")"
[[ "$ref" =~ ^[0-9a-f]{40}$ ]]
test "$(git -C "$source" rev-parse HEAD)" = "$ref"

distribution="${PACKAGE_DISTRIBUTION:-}"
if [[ -z "$distribution" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  distribution="${VERSION_CODENAME:-}"
fi
components=udp
if [[ "$distribution" != bionic ]]; then
  components="udp;http"
fi

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
  cmake "$source" -DCMAKE_BUILD_TYPE=Release "-DXGC2_XRPC_COMPONENTS=$components" -DBUILD_TESTING=OFF \
    "-DCMAKE_INSTALL_PREFIX=$prefix" "$@"
  cmake --build . -- -j"${XGC2_BUILD_JOBS:-2}"
  cmake --build . --target install
)
