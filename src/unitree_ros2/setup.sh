#!/bin/bash
echo "Setup unitree ros2 environment"
UNITREE_G1_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$UNITREE_G1_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eno1" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
