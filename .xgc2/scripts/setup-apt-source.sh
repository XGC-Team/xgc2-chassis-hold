#!/usr/bin/env bash
# Adds the signed XGC2 apt repository of this distribution to the apt sources, and the staging overlay of the release
# train when XGC2_APT_OVERLAY_URL is set. PACKAGE_DISTRIBUTION names the distribution (default: this system's codename).
set -euo pipefail
source_url="${XGC2_APT_SOURCE_URL:-https://xgc2.apt.xiaokang.ink}"
overlay_url="${XGC2_APT_OVERLAY_URL:-}"
distribution="${PACKAGE_DISTRIBUTION:-}"
if [[ -z "$distribution" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  distribution="${VERSION_CODENAME:?PACKAGE_DISTRIBUTION or VERSION_CODENAME is required}"
fi
[[ "$source_url" =~ ^https://[^[:space:]]+$ ]]
[[ -z "$overlay_url" || "$overlay_url" =~ ^https://[^[:space:]]+$ ]]
list_file=/etc/apt/sources.list.d/xgc2.list
arch="$(dpkg --print-architecture)"
install -d -m 0755 /etc/apt/keyrings
curl -fsSL --retry 5 "$source_url/xgc2-archive-keyring.gpg" -o /etc/apt/keyrings/xgc2.gpg
chmod 0644 /etc/apt/keyrings/xgc2.gpg
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/xgc2.gpg] %s %s main\n' "$arch" "$source_url" "$distribution" > "$list_file"
if [[ -n "$overlay_url" ]]; then
  printf 'deb [arch=%s signed-by=/etc/apt/keyrings/xgc2.gpg] %s %s main\n' "$arch" "${overlay_url%/}" "$distribution" >> "$list_file"
fi
apt-get update -o Dir::Etc::sourcelist="$list_file" -o Dir::Etc::sourceparts=- -o APT::Get::List-Cleanup=0
