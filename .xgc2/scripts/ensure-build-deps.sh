#!/usr/bin/env bash
# Installs the one build dependency the CI images do not carry: nlohmann-json (headers only; the library embeds
# the parts it uses). The package is named nlohmann-json-dev (2.1.1) on Ubuntu 18.04 and nlohmann-json3-dev later.
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
if ! dpkg -s "$package" >/dev/null 2>&1; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$package"
fi
