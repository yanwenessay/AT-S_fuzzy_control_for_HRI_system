#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

cat <<'EOF'

Python environment is ready.

Next steps:
1. Install Kinova Kortex Python API if it is not already installed.
2. Copy .env.example to .env and edit robot IP/login.
3. Run: python3 scripts/check_environment.py
4. Run: ./run_kinova_gen3.sh

EOF

