#!/usr/bin/env bash
# Installs the build dependencies the CI images may not carry: nlohmann-json (headers only; the library embeds the
# parts it uses) and the OpenSSL headers of the XRPC udp component. The JSON package is named nlohmann-json-dev
# (2.1.1) on Ubuntu 18.04 and nlohmann-json3-dev later.
set -euo pipefail
distribution="${PACKAGE_DISTRIBUTION:-}"
if [[ -z "$distribution" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  distribution="${VERSION_CODENAME:-}"
fi
package=nlohmann-json3-dev
if [[ "$distribution" == bionic ]]; then
  package=nlohmann-json-dev
fi
missing=()
for needed in "$package" libssl-dev; do
  if ! dpkg -s "$needed" >/dev/null 2>&1; then
    missing+=("$needed")
  fi
done
if [[ ${#missing[@]} -gt 0 ]]; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
fi
