#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail

case "${1:-}" in
    -h|--help)
        echo "Usage: docker compose run --rm --build j2534-driver"
        echo "Create the output directory first: mkdir -p build/docker"
        exit 0 ;;
    "") ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
if (( $# > 1 )); then
    echo "Expected at most one option" >&2
    exit 2
fi

output_dir=${OPENDIAG_OUTPUT_DIR:-/output}
if [[ ! -d "$output_dir" || ! -w "$output_dir" ]]; then
    echo "Create the output directory first: mkdir -p build/docker" >&2
    exit 1
fi
if [[ -e j2534_driver/build ]]; then
    echo "Use a clean container: docker compose run --rm --build j2534-driver" >&2
    exit 1
fi
export SOURCE_DATE_EPOCH
SOURCE_DATE_EPOCH=$(git log -1 --format=%ct)

make -C j2534_driver -j2 test
make -C j2534_driver CROSS_COMPILE=i686-w64-mingw32- -j2 package
archive=j2534_driver/build/win32/release/opendiag-j2534-driver.zip

staged_archive=$(mktemp "$output_dir/.opendiag-j2534-driver.XXXXXX")
trap 'rm -f -- "$staged_archive"' EXIT
cp "$archive" "$staged_archive"
chmod 644 "$staged_archive"
chown --reference="$output_dir" "$staged_archive"
mv -f -- "$staged_archive" "$output_dir/opendiag-j2534-driver.zip"
echo "Driver package: build/docker/opendiag-j2534-driver.zip"
