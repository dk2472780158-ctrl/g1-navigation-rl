#!/usr/bin/env bash
set -uo pipefail
cd /workspace/g1_walk
bash run_trial.sh --execute --speed 0.2 --seconds 3 2>&1 | tee trial_forward3s_released.log
exit "${PIPESTATUS[0]}"
