"""Headless MuJoCo sim2sim comparison for g1_walk checkpoints.

Uses the same observation, PD gains, and LSTM export path as
deploy/deploy_mujoco/deploy_mujoco.py, without the interactive viewer.
"""
import os
import sys

import copy

import numpy as np
import torch
import mujoco
import yaml

from legged_gym import LEGGED_GYM_ROOT_DIR
from rsl_rl.modules import ActorCriticRecurrent


class PolicyExporterLSTM(torch.nn.Module):
    """Same wrapper play.py uses so MuJoCo sees a stateful LSTM policy."""

    def __init__(self, actor_critic):
        super().__init__()
        self.actor = copy.deepcopy(actor_critic.actor)
        self.memory = copy.deepcopy(actor_critic.memory_a.rnn)
        self.memory.cpu()
        self.register_buffer("hidden_state", torch.zeros(self.memory.num_layers, 1, self.memory.hidden_size))
        self.register_buffer("cell_state", torch.zeros(self.memory.num_layers, 1, self.memory.hidden_size))

    def forward(self, x):
        out, (h, c) = self.memory(x.unsqueeze(0), (self.hidden_state, self.cell_state))
        self.hidden_state[:] = h
        self.cell_state[:] = c
        return self.actor(out.squeeze(0))

    def reset_memory(self):
        self.hidden_state[:] = 0.0
        self.cell_state[:] = 0.0


def gravity_orientation(quaternion):
    qw, qx, qy, qz = quaternion
    return np.array(
        [
            2 * (-qz * qx + qw * qy),
            -2 * (qz * qy + qw * qx),
            1 - 2 * (qw * qw + qz * qz),
        ],
        dtype=np.float64,
    )


def quat_to_rot(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def load_config():
    path = os.path.join(LEGGED_GYM_ROOT_DIR, "deploy/deploy_mujoco/configs/g1.yaml")
    with open(path) as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
    for key in ("policy_path", "xml_path"):
        cfg[key] = cfg[key].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
    return cfg


def load_policy(ckpt_path):
    ac = ActorCriticRecurrent(
        num_actor_obs=47,
        num_critic_obs=50,
        num_actions=12,
        actor_hidden_dims=[32],
        critic_hidden_dims=[32],
        activation="elu",
        rnn_type="lstm",
        rnn_hidden_size=64,
        rnn_num_layers=1,
        init_noise_std=0.8,
    )
    ckpt = torch.load(ckpt_path, map_location="cpu")
    ac.load_state_dict(ckpt["model_state_dict"])
    ac.eval()
    policy = PolicyExporterLSTM(ac)
    policy.eval()
    return policy


def rollout(model, cfg, policy, cmd, duration=12.0):
    sim_dt = cfg["simulation_dt"]
    decimation = cfg["control_decimation"]
    kps = np.array(cfg["kps"], dtype=np.float64)
    kds = np.array(cfg["kds"], dtype=np.float64)
    default_angles = np.array(cfg["default_angles"], dtype=np.float64)
    cmd = np.asarray(cmd, dtype=np.float64)
    cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float64)
    action_scale = cfg["action_scale"]
    num_actions = cfg["num_actions"]

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    action = np.zeros(num_actions, dtype=np.float64)
    target = default_angles.copy()
    obs = np.zeros(cfg["num_obs"], dtype=np.float32)
    if hasattr(policy, "reset_memory"):
        policy.reset_memory()

    n_steps = int(duration / sim_dt)
    warmup_steps = int(1.0 / sim_dt)
    vx, vy, wz = [], [], []
    fell_at = None
    dist = 0.0
    last_xy = data.qpos[:2].copy()

    for step in range(n_steps):
        tau = (target - data.qpos[7:]) * kps + (0.0 - data.qvel[6:]) * kds
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)

        xy = data.qpos[:2]
        dist += float(np.linalg.norm(xy - last_xy))
        last_xy = xy.copy()

        if (step + 1) % decimation == 0:
            qj = (data.qpos[7:] - default_angles) * cfg["dof_pos_scale"]
            dqj = data.qvel[6:] * cfg["dof_vel_scale"]
            quat = data.qpos[3:7]
            omega = data.qvel[3:6] * cfg["ang_vel_scale"]
            g = gravity_orientation(quat)
            t = (step + 1) * sim_dt
            phase = (t % 0.8) / 0.8
            obs[:3] = omega
            obs[3:6] = g
            obs[6:9] = cmd * cmd_scale
            obs[9 : 9 + num_actions] = qj
            obs[9 + num_actions : 9 + 2 * num_actions] = dqj
            obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action
            obs[9 + 3 * num_actions : 9 + 3 * num_actions + 2] = (
                np.sin(2 * np.pi * phase),
                np.cos(2 * np.pi * phase),
            )
            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0)).detach().numpy().squeeze()
            target = action * action_scale + default_angles

        if step >= warmup_steps:
            v_body = quat_to_rot(data.qpos[3:7]).T @ data.qvel[:3]
            vx.append(v_body[0])
            vy.append(v_body[1])
            wz.append(data.qvel[5])
            upright = gravity_orientation(data.qpos[3:7])[2] < -0.6 and data.qpos[2] > 0.45
            if fell_at is None and not upright:
                fell_at = (step + 1) * sim_dt

    vx = np.asarray(vx)
    vy = np.asarray(vy)
    wz = np.asarray(wz)
    alive = duration if fell_at is None else fell_at
    return {
        "alive_s": alive,
        "fell": fell_at is not None,
        "vx": float(np.mean(vx)) if len(vx) else 0.0,
        "vx_rmse": float(np.sqrt(np.mean((vx - cmd[0]) ** 2))) if len(vx) else 9.0,
        "vy": float(np.mean(vy)) if len(vy) else 0.0,
        "vy_rmse": float(np.sqrt(np.mean((vy - cmd[1]) ** 2))) if len(vy) else 9.0,
        "wz": float(np.mean(wz)) if len(wz) else 0.0,
        "wz_rmse": float(np.sqrt(np.mean((wz - cmd[2]) ** 2))) if len(wz) else 9.0,
        "dist_m": dist,
        "height": float(data.qpos[2]),
    }


def main():
    run_dir = os.path.join(LEGGED_GYM_ROOT_DIR, "logs/g1_walk/Oct03_16-32-54_")
    wanted = [int(x) for x in sys.argv[1:]] or [0, 250, 500, 1000, 1500, 2000]
    available = sorted(
        int(name[len("model_") : -len(".pt")])
        for name in os.listdir(run_dir)
        if name.startswith("model_") and name.endswith(".pt")
    )
    latest = available[-1]
    if latest not in wanted:
        wanted.append(latest)

    cfg = load_config()
    model = mujoco.MjModel.from_xml_path(cfg["xml_path"])
    model.opt.timestep = cfg["simulation_dt"]

    commands = {
        "fwd_0.5": [0.5, 0.0, 0.0],
        "fwd_0.8": [0.8, 0.0, 0.0],
        "turn": [0.5, 0.0, 0.4],
        "side": [0.4, 0.2, 0.0],
    }

    rows = []
    print(f"{'iter':>6} {'cmd':<8} {'alive':>6} {'vx':>6} {'vxRMSE':>7} {'vy':>6} {'wz':>6} {'dist':>6} {'fell'}")
    for it in wanted:
        path = os.path.join(run_dir, f"model_{it}.pt")
        policy = load_policy(path)
        for name, cmd in commands.items():
            stats = rollout(model, cfg, policy, cmd)
            stats.update(iter=it, cmd=name)
            rows.append(stats)
            print(
                f"{it:6d} {name:<8} {stats['alive_s']:6.2f} {stats['vx']:6.2f} {stats['vx_rmse']:7.3f} "
                f"{stats['vy']:6.2f} {stats['wz']:6.2f} {stats['dist_m']:6.2f} {int(stats['fell'])}"
            )
        sys.stdout.flush()

    print("\n# summary (lower tracking error is better; fell trials ranked by survival)")
    by_iter = {}
    for row in rows:
        by_iter.setdefault(row["iter"], []).append(row)
    ranked = []
    for it, trials in by_iter.items():
        falls = sum(t["fell"] for t in trials)
        mean_alive = float(np.mean([t["alive_s"] for t in trials]))
        fwd = next(t for t in trials if t["cmd"] == "fwd_0.5")
        ranked.append((falls, -mean_alive, fwd["vx_rmse"], it, fwd))
    ranked.sort()
    for falls, _, _, it, fwd in ranked:
        print(
            f"iter {it:4d}  falls {falls}/4  fwd0.5 vx={fwd['vx']:.2f} rmse={fwd['vx_rmse']:.3f} "
            f"alive={fwd['alive_s']:.1f}s dist={fwd['dist_m']:.2f}m"
        )


if __name__ == "__main__":
    main()
