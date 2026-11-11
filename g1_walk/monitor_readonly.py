"""Observe controller ownership, lowstate and lowcmd; never publishes motor commands."""
import json
from pathlib import Path
import time
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from dds_runtime import configure_dds_logging
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

base = Path(__file__).resolve().parent
configure_dds_logging(Path(__file__).resolve().parent)
ChannelFactoryInitialize(0,'eth0')
state = [None]
command = [None]
command_count = [0]
def receive_state(msg): state[0] = (time.monotonic(),msg)
def receive_command(msg):
    command[0] = (time.monotonic(),msg)
    command_count[0] += 1
sub = ChannelSubscriber('rt/lowstate',LowState_); sub.Init(receive_state,10)
cmdsub = ChannelSubscriber('rt/lowcmd',LowCmd_); cmdsub.Init(receive_command,10)
switch = MotionSwitcherClient(); switch.SetTimeout(3); switch.Init()
controller = switch.CheckMode()
print(json.dumps({'controller':controller,'readonly':True}),flush=True)
fsm = None
if controller[0] == 0 and controller[1].get('name') == 'ai':
    loco = LocoClient(); loco.SetTimeout(3); loco.Init()
    fsm = loco._Call(7001,'{}')
    print(json.dumps({'fsm':fsm}),flush=True)
rows = []
began = time.monotonic()
while time.monotonic()-began < 20:
    if state[0] is None:
        time.sleep(0.05); continue
    stamp,msg = state[0]
    motors = [{'index':i,'mode':m.mode,'q':m.q,'dq':m.dq,'tau_est':m.tau_est,'temperature':list(m.temperature),'vol':m.vol,'motorstate':m.motorstate,'sensor':list(m.sensor),'reserve':list(m.reserve)} for i,m in enumerate(msg.motor_state[:29])]
    row = {'t':time.monotonic()-began,'state_age':time.monotonic()-stamp,'tick':msg.tick,'mode_machine':msg.mode_machine,'mode_pr':msg.mode_pr,'rpy':list(msg.imu_state.rpy),'quaternion':list(msg.imu_state.quaternion),'motors':motors,'observed_lowcmd_count':command_count[0]}
    if command[0]:
        cs,cm=command[0]
        row['external_lowcmd']={'age':time.monotonic()-cs,'mode_machine':cm.mode_machine,'mode_pr':cm.mode_pr,'q':[m.q for m in cm.motor_cmd[:29]],'kp':[m.kp for m in cm.motor_cmd[:29]],'kd':[m.kd for m in cm.motor_cmd[:29]]}
    rows.append(row)
    print(json.dumps({'t':round(row['t'],1),'tick':msg.tick,'rpy':row['rpy'],'leg_q':[round(m['q'],3) for m in motors[:12]],'leg_mode':[m['mode'] for m in motors[:12]],'nonzero_motorstate':{m['index']:m['motorstate'] for m in motors if m['motorstate']},'max_tau_est':max(abs(m['tau_est']) for m in motors),'max_temp':max(max(m['temperature']) for m in motors),'observed_lowcmd_count':command_count[0]}),flush=True)
    time.sleep(1)
result = {'controller':controller,'fsm':fsm,'readonly':True,'rows':rows}
(base/'diagnostic_readonly.json').write_text(json.dumps(result,indent=2))
print('READONLY_DIAGNOSTIC_DONE',flush=True)
