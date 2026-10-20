#!/bin/zsh
cd "${0:A:h}" || exit 1
if [[ -x .venv/bin/python ]]; then
  exec .venv/bin/python scripts/fly_drone.py
fi
exec python3 scripts/fly_drone.py
