"""Keep SDK DDS tracing private to each deployment process."""
import os
from pathlib import Path
import unitree_sdk2py.core.channel as channel


def configure_dds_logging(base):
    directory = Path(base) / 'dds_logs'
    directory.mkdir(exist_ok=True)
    logfile = directory / f'cdds_uid{os.getuid()}_pid{os.getpid()}.log'
    # Fail before initializing DDS if this process cannot write its own trace.
    with logfile.open('a'):
        pass
    channel.ChannelConfigHasInterface = channel.ChannelConfigHasInterface.replace('/tmp/cdds.LOG', str(logfile))
    return logfile
