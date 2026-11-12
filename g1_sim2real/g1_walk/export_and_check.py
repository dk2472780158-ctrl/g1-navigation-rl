"""Export exact checkpoint with the repository's LSTM exporter; CPU only."""
import importlib.util
import json
import pathlib
import sys
import torch

root = pathlib.Path('/home/ubuntu/binjie/unitree_rl_gym')
sys.path.insert(0, str(root))
from rsl_rl.modules import ActorCriticRecurrent

out = pathlib.Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
source = (root / 'legged_gym/utils/helpers.py').read_text()
exporter_file = out / 'repository_lstm_exporter.py'
exporter_file.write_text('import os\nimport copy\nimport torch\n\n' + source[source.index('class PolicyExporterLSTM'):])
spec = importlib.util.spec_from_file_location('repository_lstm_exporter', exporter_file)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
torch.set_num_threads(1)
ac = ActorCriticRecurrent(47, 50, 12, actor_hidden_dims=[32], critic_hidden_dims=[32], activation='elu', rnn_type='lstm', rnn_hidden_size=64, rnn_num_layers=1, init_noise_std=0.8)
checkpoint = torch.load(root / 'logs/g1_walk/Oct03_16-32-54_/model_3000.pt', map_location='cpu')
ac.load_state_dict(checkpoint['model_state_dict'], strict=True)
ac.eval()
module.PolicyExporterLSTM(ac).export(str(out))
policy = torch.jit.load(str(out / 'policy_lstm_1.pt'), map_location='cpu').eval()
obs = torch.zeros(1, 47)
obs[:, 5] = -1
obs[:, 6] = 1.0
obs[:, 46] = 1
with torch.no_grad():
    reference = ac.act_inference(obs)
    action = policy(obs)
    error = (reference - action).abs().max().item()
    assert action.shape == (1, 12) and torch.isfinite(action).all()
    assert error < 1e-6
    policy.reset_memory()
    repeat = policy(obs)
    assert torch.allclose(action, repeat)
    policy.reset_memory()
    policy.save(str(out / 'policy_lstm_1.pt'))

from scripts.eval_g1_walk_sim2sim import load_config, rollout
import mujoco
cfg = load_config()
model = mujoco.MjModel.from_xml_path(cfg['xml_path'])
model.opt.timestep = cfg['simulation_dt']
results = []
for cmd in ([0, 0, 0], [0.2, 0, 0], [0.5, 0, 0], [0.8, 0, 0], [0.5, 0, 0.4], [0.4, 0.2, 0]):
    stats = rollout(model, cfg, policy, cmd, duration=12.0)
    stats['command'] = cmd
    results.append(stats)
report = {'checkpoint': 'model_3000.pt', 'iteration_field': checkpoint.get('iter'), 'torch': torch.__version__, 'mujoco': mujoco.__version__, 'actor_export_max_abs_error': error, 'reset_memory_verified': True, 'rollouts': results}
(out / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
