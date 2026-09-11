#!/usr/bin/env python3
"""Invalidate objects when the selected compiler or build flags change."""
import json
from pathlib import Path
import subprocess
import sys

path = Path(sys.argv[1])
configuration = sys.argv[2:]
for compiler in sys.argv[2:4]:
    configuration.append(subprocess.check_output([compiler, "--version"], text=True))
contents = json.dumps(configuration, indent=2) + "\n"
if not path.exists() or path.read_text() != contents:
    path.write_text(contents)
