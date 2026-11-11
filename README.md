# G1 PPO / LSTM Sim2Real

G1 walking-policy training, TorchScript export, MuJoCo evaluation and Unitree SDK2 deployment.

## Project layout

- `g1_walk/unitree_rl_gym/`: Isaac Gym training configuration and robot assets.
- `g1_walk/model_3000.pt`: training checkpoint.
- `g1_walk/policy_lstm_1.pt`, `deploy_model/walk.pt`: exported recurrent policy.
- `g1_walk/manifest.json`: training provenance and model hashes.
- `g1_walk/verification.json`: recorded export and simulation checks.
- `g1_walk/run_fixed_trial.py`: timed hardware deployment entry point.
- `src/`: SDK and ROS integration dependencies, retaining their original licenses.

## Policy interface

The recorded G1 walking configuration uses PPO with a recurrent LSTM actor, 47 actor observations, 12 leg actions and a 50 Hz policy update. The critic has privileged observations. Training is configured for 4096 parallel Isaac Gym environments. See the task configuration for reward terms and randomization.

## Documentation and dependencies

Read [the deployment manual](doc.md), [model provenance](g1_walk/manifest.json), and [source snapshot information](SOURCE.md). Deployment paths and network interfaces in the original manual are machine-specific.

The local `g1_walk/.deps` directory and Python bytecode are intentionally not distributed. Provision Python, PyTorch, the matching Unitree SDK2 Python package and its CycloneDDS dependencies for the target architecture before using deployment scripts. Follow the training repository documentation for Isaac Gym and rsl_rl dependencies. Vendored SDK libraries and robot assets are retained.

## Related projects

- [g1_nav2](https://github.com/dk2472780158-ctrl/g1_nav2): localization and navigation.
- [git_slam](https://github.com/dk2472780158-ctrl/git_slam): Mid360 / IMU processing and FAST-LIO.

The navigation velocity bridge uses the official high-level controller. It does not automatically load this PPO policy. The two control paths require separate control ownership. Publishing this repository does not validate end-to-end autonomous navigation. No hardware commands are executed during publication.
