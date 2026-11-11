#!/usr/bin/env bash
set -uo pipefail
cd /workspace/g1_walk
export PYTHONPATH="$PWD/.deps:/opt/unitree_sdk2_python"
export LD_LIBRARY_PATH=/opt/openssl11-compat:/opt/unitree_sdk2_python/cyclonedds/install/lib
export OPENBLAS_NUM_THREADS=1
/opt/miniconda3/envs/pms-py312/bin/python -u monitor_readonly.py 2>&1 | tee diagnostic_tmux.log
