#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

usage() {
    echo "Usage: docker compose run --rm --build firmware [--verify]"
    echo "Create the output directory first: mkdir -p build/docker"
    echo "--verify builds twice from scratch and compares the ZIPs."
}

verify=false
case "${1:-}" in
    --verify) verify=true ;;
    -h|--help) usage; exit 0 ;;
    "") ;;
    *) usage >&2; exit 2 ;;
esac
if (( $# > 1 )); then
    usage >&2
    exit 2
fi

if [[ $(uname -m) != x86_64 ]]; then
    echo "The release builder requires platform: linux/amd64." >&2
    exit 1
fi

if [[ ! -d .git ]] || [[ $(git rev-parse --is-shallow-repository) != false ]]; then
    echo "Use a full Git clone with tags and recursive submodules." >&2
    exit 1
fi
submodules=$(git submodule status --recursive)
if grep -q '^[-+U]' <<< "$submodules"; then
    echo "Initialize the pinned submodules: git submodule update --init --recursive" >&2
    exit 1
fi

output_dir=${OPENDIAG_OUTPUT_DIR:-/output}
if [[ ! -d "$output_dir" || ! -w "$output_dir" ]]; then
    echo "Create the output directory first: mkdir -p build/docker" >&2
    exit 1
fi
if [[ -e build ]]; then
    echo "A clean container is required: docker compose run --rm --build firmware" >&2
    exit 1
fi

export SOURCE_DATE_EPOCH
SOURCE_DATE_EPOCH=$(git log -1 --format=%ct)

cmake -S test/host -B build/host -G Ninja
cmake --build build/host --parallel 2
ctest --test-dir build/host --output-on-failure --timeout 30

cp dependencies.lock build/dependencies.lock
build_package() {
    local build_dir=$1

    # Keep developer menuconfig settings out of release builds.
    idf.py -B "$build_dir" -D "SDKCONFIG=$PWD/$build_dir/sdkconfig" \
        -D "SDKCONFIG_DEFAULTS=$PWD/sdkconfig.defaults" reconfigure
    if ! cmp -s dependencies.lock build/dependencies.lock; then
        echo "ESP-IDF changed dependencies.lock; update and commit it before building." >&2
        exit 1
    fi
    grep -qx 'CONFIG_APP_REPRODUCIBLE_BUILD=y' "$build_dir/sdkconfig"
    idf.py -B "$build_dir" firmware-package
    cmp dependencies.lock build/dependencies.lock

    (cd "$build_dir/firmware" && sha256sum --check SHA256SUMS)
    python -m zipfile --test "$build_dir/opendiag-firmware.zip"
}

build_package build/first
if "$verify"; then
    build_package build/second
    cmp build/first/opendiag-firmware.zip build/second/opendiag-firmware.zip
    echo "Reproducibility verified: both clean builds produced identical ZIPs."
fi

staged_archive=$(mktemp "$output_dir/.opendiag-firmware.XXXXXX")
trap 'rm -f -- "$staged_archive"' EXIT
cp build/first/opendiag-firmware.zip "$staged_archive"
chmod 644 "$staged_archive"
chown --reference="$output_dir" "$staged_archive"
mv -f -- "$staged_archive" "$output_dir/opendiag-firmware.zip"
echo "Firmware package: build/docker/opendiag-firmware.zip"
