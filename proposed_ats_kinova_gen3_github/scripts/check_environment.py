#!/usr/bin/env python3
"""Environment check for the Kinova Gen3 proposed AT-S controller."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path


RELEASE_DIR = Path(__file__).resolve().parents[1]
ATS_DIR = RELEASE_DIR.parent
RESPONSE1_DIR = ATS_DIR / "Response1"

REQUIRED_FILES = [
    RESPONSE1_DIR / "comparison_proposed_method_ats.py",
    ATS_DIR / "ats_imcontrol_sin_cos_jo.py",
    ATS_DIR / "Kinematic_fcn.py",
    ATS_DIR / "DiscreteIntegrator.py",
    ATS_DIR / "ts_fuzzy_output.py",
]

REQUIRED_MODULES = ["numpy", "scipy", "matplotlib"]


def check_module(name: str) -> bool:
    ok = importlib.util.find_spec(name) is not None
    print(f"{'[OK]' if ok else '[MISSING]'} Python module: {name}")
    return ok


def check_file(path: Path) -> bool:
    ok = path.exists()
    print(f"{'[OK]' if ok else '[MISSING]'} File: {path}")
    return ok


def main() -> int:
    print(f"Python: {sys.version.split()[0]}")
    print(f"Release dir: {RELEASE_DIR}")
    print(f"ATS dir: {ATS_DIR}")
    print()

    ok = True
    for module in REQUIRED_MODULES:
        ok = check_module(module) and ok

    ok = check_module("kortex_api") and ok

    print()
    for path in REQUIRED_FILES:
        ok = check_file(path) and ok

    utilities_dir = os.environ.get("KINOVA_UTILITIES_DIR", "")
    if utilities_dir:
        sys.path.insert(0, utilities_dir)
    sys.path.insert(0, str(ATS_DIR))
    sys.path.insert(0, str(RESPONSE1_DIR))
    utilities_ok = importlib.util.find_spec("utilities") is not None
    print(f"{'[OK]' if utilities_ok else '[MISSING]'} Kinova helper: utilities.py")
    ok = utilities_ok and ok

    if ok:
        print("\nEnvironment check passed.")
        return 0

    print("\nEnvironment check failed. Fix missing items before running the robot.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

