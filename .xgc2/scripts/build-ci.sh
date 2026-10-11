#!/usr/bin/env bash
# CI entry point inside a build container: makes the XRPC components and xgc2-state-machine available, packages the
# product for PACKAGE_DISTRIBUTION, installs the packages and runs the tests again against the installed result.
# Usage: build-ci.sh <scratch directory> <dependency checkouts directory>   (the directory holds xrpc/)
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
work="${1:?scratch directory}"
deps="${2:?dependency checkouts}"
mkdir -p "$work"
distribution="${PACKAGE_DISTRIBUTION:-}"
if [[ -z "$distribution" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  distribution="${VERSION_CODENAME:?PACKAGE_DISTRIBUTION or VERSION_CODENAME is required}"
fi
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
  # Release: the XRPC packages come from the release train's signed staging overlay, which the dependency set pins.
  [[ "${XGC2_DEPENDENCY_SET_DIGEST:-}" =~ ^[0-9a-f]{64}$ ]]
fi
"$root/.xgc2/scripts/ensure-build-deps.sh"

dpkg_flags=()
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
  xrpc_packages=(libxgc2-xrpc-udp-dev)
  if [[ "$distribution" != bionic ]]; then
    xrpc_packages+=(libxgc2-xrpc-dev)
  fi
  apt-get install -y --no-install-recommends "${xrpc_packages[@]}"
else
  # CI: the components of the pinned checkout, in a private prefix. They are not dpkg packages yet, so the packages
  # built here are installed without their dependency on them and the prefix is put on the library path.
  prefix="$work/xrpc"
  "$root/.xgc2/scripts/install-xrpc.sh" "$deps/xrpc" "$prefix"
  export CMAKE_PREFIX_PATH="$prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export XGC2_SHLIBDEPS_PATH="$prefix/lib"
  dpkg_flags+=(--force-depends)
fi

"$root/.xgc2/scripts/build_debs.sh" "$work/hold" "$root/.ci/debs"
dpkg "${dpkg_flags[@]}" -i "$root/.ci/debs/"*.deb
# No `cmake -S/-B` and no `--parallel`: Ubuntu 18.04 ships CMake 3.10.
mkdir -p "$work/installed"
cd "$work/installed"
cmake "$root/test" -DCMAKE_BUILD_TYPE=Debug
cmake --build . -- -j"${XGC2_BUILD_JOBS:-2}"
ctest --output-on-failure
