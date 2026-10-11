#!/usr/bin/env bash
# Installs the build dependencies the CI images may not carry: nlohmann-json (headers only; the library embeds the
# parts it uses), the OpenSSL headers of the XRPC udp component, Boost for the XRPC http component (every distribution
# but bionic), and xgc2-state-machine, which comes from the XGC2 apt repository (with the staging overlay of a release).
# The JSON package is named nlohmann-json-dev (2.1.1) on Ubuntu 18.04 and nlohmann-json3-dev later.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
distribution="${PACKAGE_DISTRIBUTION:-}"
if [[ -z "$distribution" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  distribution="${VERSION_CODENAME:-}"
fi
needed=(libssl-dev)
if [[ "$distribution" == bionic ]]; then
  needed+=(nlohmann-json-dev)
else
  needed+=(nlohmann-json3-dev libboost-dev)
fi
missing=()
for package in "${needed[@]}"; do
  if ! dpkg -s "$package" >/dev/null 2>&1; then
    missing+=("$package")
  fi
done
if [[ ${#missing[@]} -gt 0 ]]; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
fi
# The XGC2 repository (and, for a release, the overlay with the packages of the release train) for everything that
# is not a stock Ubuntu package.
"$root/.xgc2/scripts/setup-apt-source.sh"
if ! dpkg -s libxgc2-state-machine-dev >/dev/null 2>&1; then
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libxgc2-state-machine-dev
fi
