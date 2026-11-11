"""Bounded fixed-command trial using the repository's observation and SDK contract."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import torch

BASE = Path(__file__).resolve().parent
ROOT = BASE / 'unitree_rl_gym'
sys.path.insert(0, str(ROOT / 'deploy/deploy_real'))
from deploy_real import Controller
from config import Config
from common.rotation_helper import get_gravity_orientation
from common.command_helper import create_damping_cmd
from common.remote_controller import KeyMap
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from dds_runtime import configure_dds_logging
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_


class Trial(Controller):
    def __init__(self, config, raw_policy=False):
        self.raw_policy = raw_policy
        self.state_at = 0.0
        self.last_tick = None
        self.stop = threading.Event()
        self.active = False
        self.last_write = time.monotonic()
        self.previous_target = None
        self.sent = 0
        self.stage = 'preflight'
        self.owner_client = MotionSwitcherClient()
        self.owner_client.SetTimeout(0.2)
        self.owner_client.Init()
        self.records = []
        self.reason = None
        self.rejected_target = None
        self.last_observation = None
        self.lock = threading.RLock()
        tree = ET.parse(ROOT / 'resources/robots/g1_description/g1_12dof.urdf')
        joints = [j for j in tree.findall('joint') if j.get('type') == 'revolute']
        self.lower = np.array([float(j.find('limit').get('lower')) for j in joints])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in joints])
        self.effort = np.array([float(j.find('limit').get('effort')) for j in joints])
        super().__init__(config)
        self.policy.eval()
        self.policy.reset_memory()
        self.damping_cmd = unitree_hg_msg_dds__LowCmd_()
        self.damping_cmd.mode_machine = self.mode_machine_
        self.damping_cmd.mode_pr = 0
        for m in self.damping_cmd.motor_cmd:
            m.mode = 1
        create_damping_cmd(self.damping_cmd)
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()
        self.ownership_thread = threading.Thread(target=self.watch_owner, daemon=True)
        self.ownership_thread.start()

    def LowStateHgHandler(self, msg):
        super().LowStateHgHandler(msg)
        if msg.tick != self.last_tick:
            self.last_tick = msg.tick
            self.state_at = time.monotonic()

    def wait_for_low_state(self):
        deadline = time.monotonic() + 8
        while not self.state_at:
            if time.monotonic() > deadline:
                raise RuntimeError('No fresh lowstate within 8 seconds')
            time.sleep(0.01)

    def state(self):
        msg = self.low_state
        q = np.array([m.q for m in msg.motor_state[:29]], dtype=np.float32)
        dq = np.array([m.dq for m in msg.motor_state[:29]], dtype=np.float32)
        quat = np.array(msg.imu_state.quaternion, dtype=np.float32)
        omega = np.array(msg.imu_state.gyroscope, dtype=np.float32)
        if not np.isfinite(np.concatenate((q, dq, quat, omega))).all():
            raise RuntimeError('Non-finite lowstate')
        if abs(float(np.linalg.norm(quat)) - 1) > 0.05:
            raise RuntimeError('Invalid IMU quaternion')
        gravity = get_gravity_orientation(quat)
        tilt = math.acos(float(np.clip(-gravity[2], -1, 1)))
        return q, dq, omega, gravity, tilt

    def check(self):
        if self.stop.is_set():
            raise RuntimeError(self.reason or 'Stopped')
        age = time.monotonic() - self.state_at
        if age > 0.1:
            raise RuntimeError(f'Stale lowstate: {age:.3f}s')
        if self.remote_controller.button[KeyMap.select] or self.remote_controller.button[KeyMap.B]:
            raise RuntimeError('Operator Select/B stop')
        q, dq, omega, gravity, tilt = self.state()
        if not self.raw_policy and np.max(np.abs(dq[:12])) > 12:
            raise RuntimeError('Leg speed exceeds 12 rad/s')
        return q, dq, omega, gravity, tilt

    def send_cmd(self, cmd):
        with self.lock:
            self.check()
            super().send_cmd(cmd)
            self.last_write = time.monotonic()
            self.sent += 1

    def damping(self):
        with self.lock:
            super().send_cmd(self.damping_cmd)

    def watch(self):
        while not self.stop.wait(0.02):
            if not self.active:
                continue
            age = time.monotonic() - self.state_at
            gap = time.monotonic() - self.last_write
            if age > 0.1 or gap > 0.12:
                self.reason = f'Watchdog: state age {age:.3f}s, command gap {gap:.3f}s'
                self.stop.set()
                self.damping()
                break

    def watch_owner(self):
        while not self.stop.wait(0.5):
            if not self.active:
                continue
            try:
                code, owner = self.owner_client.CheckMode()
                if code != 0 or owner.get('name'):
                    self.reason = f'Controller ownership changed: {code}, {owner}'
                    self.stop.set()
            except Exception as exc:
                self.reason = f'Controller ownership check failed: {exc}'
                self.stop.set()

    def target(self, desired, q, dq, tilt):
        leg = np.asarray(desired[:12])
        if not np.isfinite(desired).all():
            raise RuntimeError('Non-finite target')
        if not self.raw_policy and (np.any(leg < self.lower) or np.any(leg > self.upper)):
            indices = np.flatnonzero((leg < self.lower) | (leg > self.upper))
            self.rejected_target = {'stage':self.stage,'targets':leg.tolist(),'q':q.tolist(),'dq':dq.tolist(),'observation':self.last_observation,'violations':[{'motor':int(i),'target':float(leg[i]),'lower':float(self.lower[i]),'upper':float(self.upper[i])} for i in indices]}
            raise RuntimeError(f'Leg target outside URDF mechanical bounds: {self.rejected_target["violations"]}')
        err = float(np.max(np.abs(leg - q[:12])))
        if not self.raw_policy and err > 0.8:
            raise RuntimeError(f'Leg target error {err:.3f} exceeds 0.8 rad')
        if not self.raw_policy and self.previous_target is not None:
            step = float(np.max(np.abs(leg - self.previous_target)))
            if step > 0.5:
                raise RuntimeError(f'Leg target jump {step:.3f} exceeds 0.5 rad')
        torque = (leg - q[:12]) * self.config.kps - dq[:12] * self.config.kds
        if not self.raw_policy and np.any(np.abs(torque) > 0.8 * self.effort):
            raise RuntimeError('Predicted leg PD torque exceeds 80% URDF effort')
        for i, motor in enumerate(self.low_cmd.motor_cmd[:29]):
            motor.q = float(desired[i])
            motor.qd = 0.0
            motor.tau = 0.0
            motor.kp = (self.config.kps + self.config.arm_waist_kps)[i]
            motor.kd = (self.config.kds + self.config.arm_waist_kds)[i]
        self.send_cmd(self.low_cmd)
        self.previous_target = leg.copy()
        self.records.append({'time': time.monotonic(), 'stage': self.stage, 'tilt': tilt, 'max_target_error': err, 'max_leg_speed': float(np.max(np.abs(dq[:12]))), 'measured_q': q.tolist(), 'measured_dq': dq.tolist(), 'target': leg.tolist()})

    def prepare(self):
        q, dq, omega, gravity, tilt = self.check()
        start = q.copy()
        default = np.concatenate((self.config.default_angles, self.config.arm_waist_target))
        code, owner = self.owner_client.CheckMode()
        if code != 0 or owner.get('name'):
            raise RuntimeError(f'Official controller must be released before actuation: {code}, {owner}')
        self.stage = 'move_to_default'
        self.active = True
        self.last_write = time.monotonic()
        began = time.monotonic()
        deadline = began
        while time.monotonic() - began < 2:
            q, dq, _, _, tilt = self.check()
            alpha = min((time.monotonic() - began) / 2, 1)
            self.target(start * (1-alpha) + default * alpha, q, dq, tilt)
            deadline += 0.02
            time.sleep(max(0, deadline - time.monotonic()))
        # Initial posture is confirmed by the on-site operator.
        # Finish the nominal transition and enter policy immediately; no pose gate.
        self.stage = 'default_pose_sent'
        q, dq, _, _, tilt = self.check()
        self.target(default, q, dq, tilt)
        return

    def walk(self, speed, seconds):
        self.stage = 'forward'
        self.policy.reset_memory()
        self.action[:] = 0
        self.counter = 0
        began = time.monotonic()
        deadline = began
        while time.monotonic() - began < seconds:
            q, dq, omega, gravity, tilt = self.check()
            self.counter += 1
            phase = (self.counter * self.config.control_dt) % 0.8 / 0.8
            self.obs[:3] = omega * self.config.ang_vel_scale
            self.obs[3:6] = gravity
            self.obs[6:9] = np.array([speed, 0, 0]) * self.config.cmd_scale
            self.obs[9:21] = (q[:12] - self.config.default_angles) * self.config.dof_pos_scale
            self.obs[21:33] = dq[:12] * self.config.dof_vel_scale
            self.obs[33:45] = self.action
            self.obs[45:47] = (np.sin(2*np.pi*phase), np.cos(2*np.pi*phase))
            self.last_observation = self.obs.tolist()
            t0 = time.monotonic()
            with torch.inference_mode():
                action = self.policy(torch.from_numpy(self.obs).unsqueeze(0)).numpy().reshape(12)
            inference_ms = (time.monotonic()-t0)*1000
            target = np.concatenate((self.config.default_angles+action*self.config.action_scale, self.config.arm_waist_target))
            self.target(target, q, dq, tilt)
            self.action = action.copy()
            self.records[-1]['observation'] = self.obs.tolist()
            self.records[-1]['inference_ms'] = inference_ms
            self.records[-1]['command'] = [speed, 0, 0]
            deadline += 0.02
            time.sleep(max(0, deadline-time.monotonic()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--raw-policy', action='store_true', help='Send original policy targets without additional joint, torque or tilt cutoffs')
    parser.add_argument('--speed', type=float, default=0.2, help='Forward velocity command in m/s; must be finite')
    parser.add_argument('--seconds', type=float, default=3.0, help='Forward command duration in seconds; must be finite and positive')
    parser.add_argument('--config', default='/workspace/deploy_model/walk.yaml', help='YAML deployment config file')
    parser.add_argument('--model', default=None, help='Optional TorchScript model path; overrides policy_path in config')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not math.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError('Speed must be finite; duration must be finite and positive')
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    config_file = Path(args.config).expanduser().resolve()
    config = Config(str(config_file))
    if args.model:
        config.policy_path = str(Path(args.model).expanduser().resolve())
    # Warm up PyTorch outside control and reset recurrent state afterward.
    model = torch.jit.load(config.policy_path).eval()
    with torch.inference_mode():
        for _ in range(10):
            result = model(torch.zeros(1,47))
            assert result.shape == (1,12) and torch.isfinite(result).all()
    configure_dds_logging(Path(__file__).resolve().parent)
    ChannelFactoryInitialize(0, 'eth0')
    trial = Trial(config, raw_policy=args.raw_policy)
    time.sleep(0.2)
    q, dq, omega, gravity, tilt = trial.check()
    info = {'config_path':str(config_file),'model_path':config.policy_path,'raw_policy':args.raw_policy,'mode': 'execute' if args.execute else 'readonly', 'q': q.tolist(), 'dq': dq.tolist(), 'tilt': tilt, 'gravity': gravity.tolist(), 'mode_machine': trial.mode_machine_, 'mode_pr': trial.low_state.mode_pr, 'torch': torch.__version__, 'numpy': np.__version__}
    print(json.dumps(info), flush=True)
    (BASE/'preflight.json').write_text(json.dumps(info,indent=2))
    if not args.execute:
        return
    completed = False
    error = None
    def on_signal(signum, frame):
        trial.reason = f'Signal {signum}'
        trial.stop.set()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,on_signal)
    try:
        trial.prepare()
        print(f'DEFAULT_POSE_SENT; operator confirms posture; commanding forward {args.speed:g} m/s for {args.seconds:g} seconds',flush=True)
        trial.walk(args.speed,args.seconds)
        completed = True
    except Exception as exc:
        error = str(exc)
        print('STOP: '+error,flush=True)
    finally:
        trial.active = False
        trial.stop.set()
        trial.watchdog.join(timeout=0.3)
        trial.ownership_thread.join(timeout=0.3)
        end = time.monotonic()+1
        while time.monotonic()<end:
            trial.damping()
            time.sleep(0.02)
        forward = [r for r in trial.records if r['stage']=='forward']
        summary = {'config_path':str(config_file),'model_path':config.policy_path,'raw_policy':args.raw_policy,'completed':completed,'error':error,'commands':trial.sent,'forward_commands':len(forward),'forward_span_s':forward[-1]['time']-forward[0]['time'] if len(forward)>1 else 0,'max_tilt':max([r['tilt'] for r in trial.records],default=tilt),'max_target_error':max([r['max_target_error'] for r in trial.records],default=0),'records':trial.records,'damping_sent_for_s':1,'actual_displacement':'not measured','rejected_target':trial.rejected_target}
        stamp = time.strftime('%Y%m%d_%H%M%S')
        (BASE/f'trial_result_{stamp}.json').write_text(json.dumps(summary,indent=2))
        (BASE/'trial_result.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps({k:v for k,v in summary.items() if k!='records'}),flush=True)
    sys.exit(0 if completed else 2)

if __name__=='__main__':
    main()
