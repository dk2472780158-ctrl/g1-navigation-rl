#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONPATH="$PWD/.deps:$PWD/unitree_rl_gym:/opt/unitree_sdk2_python"
export CYCLONEDDS_HOME=/opt/unitree_sdk2_python/cyclonedds/install
export LD_LIBRARY_PATH="/opt/openssl11-compat:$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /opt/miniconda3/envs/pms-py312/bin/python -u run_fixed_trial.py "$@"
