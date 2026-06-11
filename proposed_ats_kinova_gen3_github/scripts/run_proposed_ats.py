#!/usr/bin/env python3
"""Launcher for Response1/comparison_proposed_method_ats.py.

The original controller is kept in place. This wrapper only fixes the Python
path and forwards command-line arguments to the original script.
"""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


RELEASE_DIR = Path(__file__).resolve().parents[1]
ATS_DIR = RELEASE_DIR.parent
RESPONSE1_DIR = ATS_DIR / "Response1"
CONTROLLER = RESPONSE1_DIR / "comparison_proposed_method_ats.py"


def main() -> int:
    if not CONTROLLER.exists():
        print(f"Controller not found: {CONTROLLER}", file=sys.stderr)
        return 1

    utilities_dir = os.environ.get("KINOVA_UTILITIES_DIR", "")
    for path in [utilities_dir, str(RESPONSE1_DIR), str(ATS_DIR), str(ATS_DIR.parent)]:
        if path and path not in sys.path:
            sys.path.insert(0, path)

    sys.argv = [str(CONTROLLER), *sys.argv[1:]]
    runpy.run_path(str(CONTROLLER), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

