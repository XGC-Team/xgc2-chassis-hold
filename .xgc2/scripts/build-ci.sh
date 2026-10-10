#!/usr/bin/env bash
# CI entry point inside a build container: makes the XRPC udp component available, packages the product for
# PACKAGE_DISTRIBUTION, installs the packages and runs the tests again against the installed result.
# Usage: build-ci.sh <scratch directory> <dependency checkouts directory>   (the directory holds xrpc/)
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
work="${1:?scratch directory}"
deps="${2:?dependency checkouts}"
mkdir -p "$work"
"$root/.xgc2/scripts/ensure-build-deps.sh"

dpkg_flags=()
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
  # Release: the XRPC udp package comes from the release train's signed staging overlay.
  [[ "${XGC2_DEPENDENCY_SET_DIGEST:-}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$XGC2_APT_OVERLAY_URL" =~ ^https://[^[:space:]]+$ ]]
  distribution="${PACKAGE_DISTRIBUTION:?PACKAGE_DISTRIBUTION is required}"
  install -d -m 0755 /etc/apt/keyrings
  curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg -o /etc/apt/keyrings/xgc2.gpg
  printf 'deb [signed-by=/etc/apt/keyrings/xgc2.gpg] %s '"$distribution"' main\n' \
    https://xgc2.apt.xiaokang.ink "${XGC2_APT_OVERLAY_URL%/}" > /etc/apt/sources.list.d/xgc2.list
  apt-get update -o Dir::Etc::sourcelist=sources.list.d/xgc2.list -o Dir::Etc::sourceparts=-
  apt-get install -y --no-install-recommends libxgc2-xrpc-udp-dev
else
  # CI: the udp component of the pinned checkout, in a private prefix. It is not a dpkg package, so the packages
  # built here are installed without their dependency on it and the prefix is put on the library path.
  prefix="$work/xrpc"
  "$root/.xgc2/scripts/install-xrpc-udp.sh" "$deps/xrpc" "$prefix"
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
