"""Validate DDS initialization and lowstate without publishing motor commands."""
from pathlib import Path
import json
import time
from dds_runtime import configure_dds_logging
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
log=configure_dds_logging(Path(__file__).resolve().parent)
ChannelFactoryInitialize(0,'eth0')
latest=[None]
def receive(msg): latest[0]=msg
sub=ChannelSubscriber('rt/lowstate',LowState_); sub.Init(receive,10)
deadline=time.monotonic()+5
while latest[0] is None and time.monotonic()<deadline: time.sleep(0.01)
assert latest[0] is not None,'No lowstate received'
client=MotionSwitcherClient();client.SetTimeout(2);client.Init()
print(json.dumps({'dds_initialized':True,'tick':latest[0].tick,'controller':client.CheckMode(),'logfile':str(log),'readonly':True}))
