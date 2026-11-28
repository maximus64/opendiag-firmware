#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Package one Unix driver build as <out_dir>/<name>.zip with a SHA256SUMS file beside it.
# The ZIP is flat, like the Windows package from tools/package.py, and carries its own SHA256SUMS
# so the files can be checked after extraction.
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
(cd "$stage" && sha256sum libopendiag.so libopendiag0404.so opendiag.ini > SHA256SUMS)
# Fixed entry timestamps keep the ZIP reproducible, as the tar's --sort and --owner flags did.
touch -d "@${SOURCE_DATE_EPOCH:-315532800}" "$stage"/*
rm -f "$out_dir/$name.zip"
zip -X -j -q "$out_dir/$name.zip" "$stage/libopendiag.so" "$stage/libopendiag0404.so" "$stage/opendiag.ini" "$stage/SHA256SUMS"
rm -rf "$stage"
(cd "$out_dir" && sha256sum "$name.zip" > SHA256SUMS)
