#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

echo "Running Robot Arm System"

VENV="../myenv3.10/bin/activate"
if [ ! -f "$VENV" ]; then
    echo "ERROR: Virtual environment not found at $VENV"
    echo "Create it with: python3.10 -m venv ../myenv3.10"
    exit 1
fi

# shellcheck source=/dev/null
source "$VENV"

exec python main.py "$@"
