#!/usr/bin/env bash
# =============================================================================
# Run the calibration with the experiment config.
#
# Backs up the current config/params.yaml, swaps in the experiment config,
# runs the calibrator from the repo root, captures the full console log to
# experiments/output/calibrator_output.txt, then restores the original config.
#
# Outputs:
#   experiments/output/calibrator_output.txt        full stdout/stderr log
#   experiments/output/extrinsic_parameters.txt     top-relative result
# =============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"

CONFIG="config/params.yaml"
EXP_CONFIG="experiments/config/params.yaml"
BIN="build/bin/ml_gins_calibrator"
OUTDIR="experiments/output"
LOG="$OUTDIR/calibrator_output.txt"
BACKUP="$OUTDIR/params.yaml.bak"

[ -x "$BIN" ] || { echo "ERROR: $BIN not found. Build first (scripts/build.sh)."; exit 1; }
mkdir -p "$OUTDIR"

cp "$CONFIG" "$BACKUP"
cp "$EXP_CONFIG" "$CONFIG"
echo "[config] using $EXP_CONFIG (backup: $BACKUP)"

restore() { cp "$BACKUP" "$CONFIG"; echo "[config] restored $CONFIG"; }
trap restore EXIT

echo "[run] $BIN  (log -> $LOG)"
echo "      full 3-LiDAR + Multi-LiDAR run can take several hours on 4 threads."
"./$BIN" 2>&1 | tee "$LOG"
echo "[done] result -> $OUTDIR/extrinsic_parameters.txt"
