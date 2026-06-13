#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -f ".env" ]; then
  set -a
  # shellcheck disable=SC1091
  source ".env"
  set +a
fi

KINOVA_IP="${KINOVA_IP:-}"
KINOVA_USERNAME="${KINOVA_USERNAME:-}"
KINOVA_PASSWORD="${KINOVA_PASSWORD:-}"

if [ -z "$KINOVA_IP" ] || [ -z "$KINOVA_USERNAME" ] || [ -z "$KINOVA_PASSWORD" ]; then
  cat <<'EOF'
Missing Kinova connection settings.
Create .env from .env.example and set your own robot IP, username, and password.
EOF
  exit 1
fi

python3 scripts/run_proposed_ats.py \
  --ip "$KINOVA_IP" \
  -u "$KINOVA_USERNAME" \
  -p "$KINOVA_PASSWORD" \
  "$@"
