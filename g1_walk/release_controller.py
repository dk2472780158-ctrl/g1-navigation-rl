"""Release official motion ownership for the operator-confirmed suspended robot."""
import json
from pathlib import Path
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from dds_runtime import configure_dds_logging
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
configure_dds_logging(Path(__file__).resolve().parent)
ChannelFactoryInitialize(0,'eth0')
counts = [0]
def receive(msg): counts[0] += 1
sub=ChannelSubscriber('rt/lowcmd',LowCmd_); sub.Init(receive,10)
switch=MotionSwitcherClient(); switch.SetTimeout(3); switch.Init()
before=switch.CheckMode(); print(json.dumps({'before':before}),flush=True)
if before[0]!=0 or before[1].get('name') not in ('ai',''):
    raise RuntimeError('Unexpected owner; no release performed')
code=0
if before[1].get('name'):
    code=switch.ReleaseMode()
    print(json.dumps({'release_return':code}),flush=True)
checks=[]
for i in range(6):
    time.sleep(0.5)
    checks.append(switch.CheckMode())
    print(json.dumps({'after':checks[-1],'lowcmd_received':counts[0]}),flush=True)
if any(code!=0 or data.get('name') for code,data in checks):
    raise RuntimeError('Official controller did not stay released')
start=counts[0]; time.sleep(1); residual=counts[0]-start
result={'before':before,'release_return':code,'after':checks,'last_second_external_lowcmd_count':residual}
Path('/workspace/g1_walk/controller_release.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result),flush=True)
