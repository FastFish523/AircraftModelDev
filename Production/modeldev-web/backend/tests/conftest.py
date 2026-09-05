from __future__ import annotations

import sys
from pathlib import Path


# `modeldev-web` contains a hyphen, so add it to sys.path and import the
# conventional `backend` package beneath it.
MODELDEV_WEB_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(MODELDEV_WEB_DIR))
