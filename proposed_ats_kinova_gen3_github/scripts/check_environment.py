#!/usr/bin/env python3
"""Environment check for the bundled Kinova Gen3 AT-S controller."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

RELEASE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = RELEASE_DIR / "src"
REQUIRED_FILES = [
    "comparison_proposed_method_ats.py",
    "ats_imcontrol_sin_cos_jo.py",
    "Kinematic_fcn.py",
    "DiscreteIntegrator.py",
    "ts_fuzzy_output.py",
    "fuzzy_membership_fcn.py",
    "fuzzyoutput.py",
    "control_main.py",
    "utilities.py",
]
REQUIRED_MODULES = ["numpy", "scipy", "matplotlib", "kortex_api"]

def main() -> int:
    print(f"Python: {sys.version.split()[0]}")
    print(f"Release dir: {RELEASE_DIR}")
    print(f"Source dir: {SRC_DIR}")
    ok = True
    for name in REQUIRED_MODULES:
        found = importlib.util.find_spec(name) is not None
        print(f"{'[OK]' if found else '[MISSING]'} Python module: {name}")
        ok = ok and found
    for name in REQUIRED_FILES:
        path = SRC_DIR / name
        found = path.exists()
        print(f"{'[OK]' if found else '[MISSING]'} File: {path}")
        ok = ok and found
    return 0 if ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
