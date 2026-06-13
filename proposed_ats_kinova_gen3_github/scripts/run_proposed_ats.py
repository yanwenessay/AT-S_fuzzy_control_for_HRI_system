#!/usr/bin/env python3
"""Launch the bundled AT-S impedance controller."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

RELEASE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = RELEASE_DIR / "src"
CONTROLLER = SRC_DIR / "comparison_proposed_method_ats.py"

def main() -> int:
    if not CONTROLLER.exists():
        print(f"Controller not found: {CONTROLLER}", file=sys.stderr)
        return 1
    for path in (str(SRC_DIR), str(RELEASE_DIR)):
        if path not in sys.path:
            sys.path.insert(0, path)
    sys.argv = [str(CONTROLLER), *sys.argv[1:]]
    runpy.run_path(str(CONTROLLER), run_name="__main__")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
