#!/usr/bin/env bash
# Build, test and package against the installed toolchain. Outputs are never overwritten.
# Usage: build_debs.sh <fresh work directory> <output directory>
# PACKAGE_DISTRIBUTION names the distribution the packages are for (default: this system's codename).
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
distribution="${PACKAGE_DISTRIBUTION:-}"
if [[ -z "$distribution" ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  distribution="${VERSION_CODENAME:?PACKAGE_DISTRIBUTION or VERSION_CODENAME is required}"
fi
# The version of this distribution's packages is the product version with the distribution suffix.
version="$(awk -v key="$distribution:" '$1 == key { print $2; exit }' "$root/.xgc2/product.yml")"
test -n "$version"
dpkg --validate-version "$version"
test "$version" = "$(sed -n 's/^version:[[:space:]]*//p' "$root/.xgc2/product.yml" | head -n 1)~$distribution"
runtime_package=libxgc2-chassis-hold1
dev_package=libxgc2-chassis-hold-dev
for package in "$runtime_package" "$dev_package"; do
  test ! -e "$out/${package}_${version}_${arch}.deb"
done

cmake_args=("-DCMAKE_BUILD_TYPE=RelWithDebInfo" "-DCMAKE_INSTALL_PREFIX=/usr" "-DCMAKE_INSTALL_LIBDIR=lib/$multiarch")
# Warnings are errors only with the compiler the code is verified with in CI; newer ones report on their own.
if [[ "$distribution" == focal ]]; then
  cmake_args+=("-DXGC2_CHASSIS_HOLD_WERROR=ON")
fi
# No `cmake -S/-B` and no `--parallel`: Ubuntu 18.04 ships CMake 3.10.
mkdir -p "$work/build"
(cd "$work/build" && cmake "$root" "${cmake_args[@]}")
cmake --build "$work/build" -- -j"${XGC2_BUILD_JOBS:-2}"
(cd "$work/build" && ctest --output-on-failure)
DESTDIR="$work/stage" cmake --build "$work/build" --target install

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
find "$runtime/usr/lib/$multiarch" -type f -name '*.so.*' -exec strip --strip-unneeded {} +
printf 'libxgc2_chassis_hold 1 %s (>= %s)\n' "$runtime_package" "$version" > "$runtime/DEBIAN/shlibs"
mkdir -p "$work/debian"
printf 'Source: xgc2-chassis-hold\nSection: libs\nPriority: optional\nMaintainer: XGC Team <apt@example.com>\n\nPackage: %s\nArchitecture: any\nDescription: Chassis HOLD\n' \
  "$runtime_package" > "$work/debian/control"
cp "$runtime/DEBIAN/shlibs" "$work/debian/shlibs.local"
pushd "$work" >/dev/null
mapfile -t libraries < <(find "$runtime/usr/lib/$multiarch" -type f -name '*.so.*' | sort)
depends="$(dpkg-shlibdeps -O -x"$runtime_package" -l"$runtime/usr/lib/$multiarch" "${libraries[@]/#/-e}" | sed -n 's/^shlibs:Depends=//p')"
popd >/dev/null
test -n "$depends"
cat > "$runtime/DEBIAN/control" <<CONTROL
Package: $runtime_package
Version: $version
Architecture: $arch
Section: libs
Priority: optional
Maintainer: XGC Team <apt@example.com>
Depends: $depends
Description: XGC2 chassis HOLD runtime
 Domain library and transport-neutral JSON service for chassis command gating.
CONTROL
printf 'activate-noawait ldconfig\n' > "$runtime/DEBIAN/triggers"
cat > "$dev/DEBIAN/control" <<CONTROL
Package: $dev_package
Version: $version
Architecture: $arch
Section: libdevel
Priority: optional
Maintainer: XGC Team <apt@example.com>
Depends: $runtime_package (= $version)
Description: XGC2 chassis HOLD development files
 C++17 headers, CMake package XgcChassisHold (component core) and the HOLD contract.
CONTROL
for spec in "runtime $runtime_package" "dev $dev_package"; do
  read -r dir package <<<"$spec"
  mkdir -p "$work/$dir/usr/share/doc/$package"
  cp "$root/LICENSE" "$work/$dir/usr/share/doc/$package/copyright"
  dpkg-deb --root-owner-group --build "$work/$dir" "$work/$package.deb"
  cp --no-clobber "$work/$package.deb" "$out/${package}_${version}_${arch}.deb"
  cmp "$work/$package.deb" "$out/${package}_${version}_${arch}.deb"
done
