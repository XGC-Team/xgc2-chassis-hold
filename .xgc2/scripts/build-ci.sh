#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
work="${1:?scratch directory}"
deps="${2:?exact source dependency checkouts}"
mkdir -p "$work"
test "$(git -C "$deps/xrpc" rev-parse HEAD)" = f8a9258f7d3c395ccdef27405cccadceb794884c
test "$(git -C "$deps/state-machine" rev-parse HEAD)" = 846a75c798b8722d7b5af7d569378e092340dc19
export CC=clang-10 CXX=clang++-10
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
  [[ "${XGC2_DEPENDENCY_SET_DIGEST:-}" =~ ^[0-9a-f]{64}$ ]]
  [[ "$XGC2_APT_OVERLAY_URL" =~ ^https://[^[:space:]]+$ ]]
  install -d -m 0755 /etc/apt/keyrings
  curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg -o /etc/apt/keyrings/xgc2.gpg
  printf 'deb [signed-by=/etc/apt/keyrings/xgc2.gpg] %s focal main\n' "${XGC2_APT_OVERLAY_URL%/}" > /etc/apt/sources.list.d/xgc2.list
  apt-get update -o Dir::Etc::sourcelist=sources.list.d/xgc2.list -o Dir::Etc::sourceparts=-
  apt-get install -y --no-install-recommends libxgc2-xrpc-dev libxgc2-state-machine-dev
else
  python3 "$deps/xrpc/tools/build-packages.py" --output "$work/xrpc" \
    --distribution focal --architecture "$(dpkg --print-architecture)" --language cpp --cpp-profile http --cxx "$CXX"
  PACKAGE_DISTRIBUTION=focal "$deps/state-machine/.xgc2/scripts/build_deb.sh"
  dpkg -i "$work/xrpc/"*.deb "$deps/state-machine/.ci/debs/"*.deb
fi
"$root/.xgc2/scripts/build_debs.sh" "$work/hold" "$root/.ci/debs"
dpkg -i "$root/.ci/debs/"*.deb
cmake -S "$root/test" -B "$work/installed" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$work/installed" --parallel 2
(cd "$work/installed" && ctest --output-on-failure)
