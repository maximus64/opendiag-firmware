#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
PYTHON=${PYTHON:-.venv/bin/python}
"$PYTHON" -m nanopb.generator.nanopb_generator -I protocol -D main protocol/j2534.proto
protoc -I protocol --python_out=protocol protocol/j2534.proto
