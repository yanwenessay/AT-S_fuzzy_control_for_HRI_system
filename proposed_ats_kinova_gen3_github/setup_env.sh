#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

cat <<'EOF'
Environment is ready.
Install the official Kinova Kortex Python API separately, then run:
  python3 scripts/check_environment.py
EOF
