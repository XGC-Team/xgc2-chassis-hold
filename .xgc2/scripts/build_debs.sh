#!/usr/bin/env bash
# Build and test against installed SDKs. Outputs are never overwritten.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(git -C "$root" show -s --format=%ct HEAD)}"
[[ "$SOURCE_DATE_EPOCH" =~ ^[0-9]+$ ]]
work="${1:?fresh work directory}"
out="${2:?output directory}"
test ! -e "$work"
mkdir -p "$work" "$out"
work="$(realpath "$work")"
out="$(realpath "$out")"
arch="$(dpkg --print-architecture)"
multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
version="$(awk '$1 == "focal:" { print $2; exit }' "$root/.xgc2/product.yml")"
test -n "$version"
dpkg --validate-version "$version"
for package in libxgc2-chassis-hold0 libxgc2-chassis-hold-dev; do
  test ! -e "$out/${package}_${version}_${arch}.deb"
done
cmake -S "$root" -B "$work/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR="lib/$multiarch"
cmake --build "$work/build" --parallel 2
(cd "$work/build" && ctest --output-on-failure)
DESTDIR="$work/stage" cmake --install "$work/build"
runtime="$work/runtime"
dev="$work/dev"
mkdir -p "$runtime/DEBIAN" "$runtime/usr/lib/$multiarch" "$dev/DEBIAN" "$dev/usr/lib/$multiarch"
cp -a "$work/stage/usr/include" "$dev/usr/"
cp -a "$work/stage/usr/share" "$dev/usr/"
cp -a "$work/stage/usr/lib/$multiarch/cmake" "$dev/usr/lib/$multiarch/"
for file in "$work/stage/usr/lib/$multiarch/"*.so*; do
  if [[ "$file" == *.so ]]; then cp -a "$file" "$dev/usr/lib/$multiarch/";
  else cp -a "$file" "$runtime/usr/lib/$multiarch/"; fi
done
for component in state rpc_handler server; do
  printf 'libxgc2_chassis_hold_%s 0 libxgc2-chassis-hold0 (>= %s)\n' "$component" "$version"
done > "$runtime/DEBIAN/shlibs"
mkdir -p "$work/debian"
printf 'Source: xgc2-chassis-hold\nSection: libs\nPriority: optional\nMaintainer: XGC Team <apt@example.com>\n\nPackage: libxgc2-chassis-hold0\nArchitecture: any\nDescription: Chassis command gating\n' > "$work/debian/control"
cat "$runtime/DEBIAN/shlibs" > "$work/debian/shlibs.local"
# The existing state-machine product ships its runtime in the dev package.
printf 'libxgc2_state_machine 0 libxgc2-state-machine-dev (>= 0.1.3-12~focal)\n' >> "$work/debian/shlibs.local"
pushd "$work" >/dev/null
mapfile -t libraries < <(find "$runtime/usr/lib/$multiarch" -type f -name '*.so.*' | sort)
depends="$(dpkg-shlibdeps -O -xlibxgc2-chassis-hold0 -l"$runtime/usr/lib/$multiarch" "${libraries[@]/#/-e}" | sed -n 's/^shlibs:Depends=//p')"
popd >/dev/null
test -n "$depends"
cat > "$runtime/DEBIAN/control" <<CONTROL
Package: libxgc2-chassis-hold0
Version: $version
Architecture: $arch
Section: libs
Priority: optional
Maintainer: XGC Team <apt@example.com>
Depends: $depends
Description: XGC2 chassis command gating runtime
 State-machine regions, bounded RPC handler and standalone RPC host.
CONTROL
printf 'activate-noawait ldconfig\n' > "$runtime/DEBIAN/triggers"
cat > "$dev/DEBIAN/control" <<CONTROL
Package: libxgc2-chassis-hold-dev
Version: $version
Architecture: $arch
Section: libdevel
Priority: optional
Maintainer: XGC Team <apt@example.com>
Depends: libxgc2-chassis-hold0 (= $version), libxgc2-xrpc-dev (>= 0.1.0-2~focal)
Breaks: libxgc2-robotics-interfaces-dev (<< 0.2.0-3~focal)
Replaces: libxgc2-robotics-interfaces-dev (<< 0.2.0-3~focal)
Description: XGC2 chassis command gating development library
 C++20 State, Rpc and Server components with explicit host ownership.
CONTROL
for spec in "runtime libxgc2-chassis-hold0" "dev libxgc2-chassis-hold-dev"; do
  read -r dir package <<<"$spec"
  mkdir -p "$work/$dir/usr/share/doc/$package"
  cp "$root/LICENSE" "$work/$dir/usr/share/doc/$package/copyright"
  dpkg-deb --root-owner-group --build "$work/$dir" "$work/$package.deb"
  cp --no-clobber "$work/$package.deb" "$out/${package}_${version}_${arch}.deb"
  cmp "$work/$package.deb" "$out/${package}_${version}_${arch}.deb"
done
