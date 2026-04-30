from __future__ import annotations

import sys
from pathlib import Path


SENSOR_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = SENSOR_DIR.parents[1]

for path in (str(SENSOR_DIR), str(WORKSPACE_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)
