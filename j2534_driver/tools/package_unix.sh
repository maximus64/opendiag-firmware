#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Package one Unix driver build as <out_dir>/<name>.tar.gz with a SHA256SUMS file beside it.
# Usage: sh tools/package_unix.sh <build_dir> <out_dir> <name>
set -eu

if [ "$#" -ne 3 ]; then
    echo "Usage: sh tools/package_unix.sh <build_dir> <out_dir> <name>" >&2
    exit 2
fi
build_dir=$1
out_dir=$2
name=$3

stage="$out_dir/$name"
mkdir -p "$stage"
cp "$build_dir/libopendiag.so" "$build_dir/libopendiag0404.so" "$build_dir/opendiag.ini" "$stage/"
tar --sort=name --owner=0 --group=0 --numeric-owner -C "$out_dir" -czf "$out_dir/$name.tar.gz" "$name"
rm -rf "$stage"
(cd "$out_dir" && sha256sum "$name.tar.gz" > SHA256SUMS)
