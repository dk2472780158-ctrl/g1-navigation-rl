#!/usr/bin/env bash
# G1 行走训练环境（国内镜像友好）。需 NVIDIA GPU + 驱动 >= 525。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ROOT="${ISAAC_GYM_ROOT:-$HOME/binjie/isaacgym}"
RSL_RL_ROOT="${RSL_RL_ROOT:-$HOME/binjie/rsl_rl}"
ENV_NAME="${CONDA_ENV_NAME:-unitree-rl}"

export PATH="${HOME}/miniforge3/bin:${HOME}/binjie/miniconda3/bin:${PATH}"

echo "==> Conda 环境: ${ENV_NAME}"
if ! conda env list | grep -qE "^${ENV_NAME}[[:space:]]"; then
  conda create -y -n "${ENV_NAME}" python=3.8
fi
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "${ENV_NAME}"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
if [[ "$(python -c 'import sys; print(sys.version_info[:2])')" != "(3, 8)" ]]; then
  echo "错误: 未进入 ${ENV_NAME}（当前 $(python --version)）。请先: source \"\$(conda info --base)/etc/profile.d/conda.sh\" && conda activate ${ENV_NAME}"
  exit 1
fi

pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple 2>/dev/null || true

if python - <<'PY' 2>/dev/null; then
import torch
assert torch.__version__.startswith("2.3")
assert torch.cuda.is_available()
PY
  echo "==> PyTorch 已就绪: $(python -c 'import torch; print(torch.__version__, \"cuda\", torch.cuda.is_available())')"
else
  echo "==> PyTorch (CUDA 12.1, conda — 国内比 pip 更稳)"
  rm -rf "${CONDA_PKGS_DIRS:-$HOME/miniforge3/pkgs}"/pytorch-2.3.1-py3.8_cuda12.1_cudnn8.9.2_0 2>/dev/null || true
  # 仅用 pytorch/nvidia/conda-forge，避免 default/清华 main 混源导致 openmp 冲突
  conda install -y pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 \
    pytorch-cuda=12.1 --override-channels \
    -c pytorch -c nvidia -c conda-forge \
  || {
    echo "conda 安装失败时，可删损坏缓存: rm -rf ~/miniforge3/pkgs/pytorch-2.3.1-* && conda clean -y --packages"
    exit 1
  }
fi

if [[ ! -d "${ISAAC_ROOT}/python" ]]; then
  echo "==> 下载 Isaac Gym Preview 4（走 NVIDIA 国内 CDN）"
  mkdir -p "$(dirname "${ISAAC_ROOT}")"
  tmp="$(mktemp)"
  wget -q --show-progress -O "${tmp}" "https://developer.nvidia.com/isaac-gym-preview-4"
  tar -xzf "${tmp}" -C "$(dirname "${ISAAC_ROOT}")"
  rm -f "${tmp}"
  # tarball 解压目录名一般为 isaacgym
  if [[ ! -d "${ISAAC_ROOT}/python" ]] && [[ -d "$(dirname "${ISAAC_ROOT}")/isaacgym/python" ]]; then
    ISAAC_ROOT="$(dirname "${ISAAC_ROOT}")/isaacgym"
  fi
fi

echo "==> 安装 Isaac Gym: ${ISAAC_ROOT}"
pip install -e "${ISAAC_ROOT}/python"

if [[ ! -d "${RSL_RL_ROOT}/.git" ]]; then
  echo "==> 克隆 rsl_rl v1.0.2"
  git clone --depth 1 --branch v1.0.2 \
    https://ghfast.top/https://github.com/leggedrobotics/rsl_rl.git "${RSL_RL_ROOT}" \
    || git clone --depth 1 --branch v1.0.2 \
    https://github.com/leggedrobotics/rsl_rl.git "${RSL_RL_ROOT}"
fi
pip install -e "${RSL_RL_ROOT}"

echo "==> 安装 unitree_rl_gym 及依赖"
pip install -e "${ROOT}" --no-deps
pip install matplotlib "numpy==1.20" tensorboard "mujoco==3.2.3" pyyaml

echo "==> 验证 import"
python - <<'PY'
import isaacgym
import torch
import rsl_rl
import legged_gym
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
print("legged_gym OK")
PY

echo ""
echo "训练（平地行走）:"
echo "  conda activate ${ENV_NAME}"
echo "  cd ${ROOT}"
echo "  python legged_gym/scripts/train.py --task=g1_walk --headless --num_envs=4096"
echo ""
echo "Sim2Sim (MuJoCo):"
echo "  python deploy/deploy_mujoco/deploy_mujoco.py g1.yaml"
echo ""
echo "Sim2Real (G1 Edu 29): 先 pip install unitree_sdk2py，再:"
echo "  python deploy/deploy_real/deploy_real.py <网卡名> g1.yaml"
