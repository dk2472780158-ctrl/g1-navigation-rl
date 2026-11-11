"""Exercise target guards with a fake publisher; sends no DDS commands."""
import run_fixed_trial as mod
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as ET
trial = mod.Trial.__new__(mod.Trial)
trial.raw_policy = False
trial.last_observation = None
trial.config = mod.Config('/workspace/deploy_model/walk.yaml')
trial.low_cmd = mod.unitree_hg_msg_dds__LowCmd_()
joints = [j for j in ET.parse(mod.ROOT/'resources/robots/g1_description/g1_12dof.urdf').findall('joint') if j.get('type')=='revolute']
trial.lower = np.array([float(j.find('limit').get('lower')) for j in joints])
trial.upper = np.array([float(j.find('limit').get('upper')) for j in joints])
trial.effort = np.array([float(j.find('limit').get('effort')) for j in joints])
trial.previous_target = None
trial.records = []
trial.stage = 'offline'
published = []
trial.send_cmd = lambda cmd: published.append([m.q for m in cmd.motor_cmd[:29]])
default = np.concatenate((trial.config.default_angles,trial.config.arm_waist_target))
trial.target(default, default, np.zeros(29), 0)
assert len(published)==1 and np.allclose(published[0],default)
cases = []
bad = default.copy(); bad[0] = np.nan; cases.append(('Non-finite',bad,default,np.zeros(29)))
bad = default.copy(); bad[0] = trial.upper[0]+0.1; cases.append(('mechanical',bad,default,np.zeros(29)))
badq = default.copy(); badq[0] += 0.81; cases.append(('target error',default,badq,np.zeros(29)))
bad = default.copy(); bad[0] += 0.51; cases.append(('jump',bad,default,np.zeros(29)))
baddq = np.zeros(29); baddq[0] = 100; cases.append(('torque',default,default,baddq))
for expected, desired, q, dq in cases:
    try: trial.target(desired,q,dq,0)
    except RuntimeError as exc: assert expected in str(exc),(expected,str(exc))
    else: raise AssertionError(expected)
assert len(published)==1, 'Rejected targets reached fake publisher'
print('Offline guards passed: valid target, finite values, bounds, tracking error, target jump, PD torque; no DDS publisher used.')
class Occupied:
    def CheckMode(self): return 0, {'name':'ai'}
trial.owner_client=Occupied()
trial.check=lambda: (default,np.zeros(29),np.zeros(3),np.array([0,0,-1]),0)
try: trial.prepare()
except RuntimeError as exc: assert 'must be released' in str(exc)
else: raise AssertionError('Occupied controller did not prevent actuation')
assert len(published)==1
print('Controller ownership guard passed: ai owner prevents all preparation commands.')

trial.raw_policy=True
outside=default.copy(); outside[4]=trial.upper[4]+0.2
trial.target(outside,default,np.zeros(29),0.5)
assert np.allclose(published[-1],outside), 'Raw policy target was changed'
assert len(published)==2
print('Raw policy passthrough verified: original target preserved without position, jump, tracking or torque cutoff.')
