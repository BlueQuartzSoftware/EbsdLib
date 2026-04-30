#!/usr/bin/env zsh
# Run mtex_pole_figure_positions.m headlessly with MTEX initialized.

set -euo pipefail

MATLAB_BIN="/Applications/MATLAB_R2025b.app/bin/matlab"
MTEX_STARTUP="/Users/mjackson/Workspace7/mtex-6.1.0/startup_mtex.m"
SCRIPT_DIR="${0:A:h}"
PF_SCRIPT="${SCRIPT_DIR}/mtex_pole_figure_positions.m"

for f in "$MATLAB_BIN" "$MTEX_STARTUP" "$PF_SCRIPT"; do
  if [[ ! -e "$f" ]]; then
    print -u2 "Error: required file not found: $f"
    exit 1
  fi
done

"$MATLAB_BIN" -batch "run('${MTEX_STARTUP}'); run('${PF_SCRIPT}');"
