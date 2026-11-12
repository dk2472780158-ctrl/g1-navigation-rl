#!/bin/bash
echo "Setup unitree ros2 environment with default interface"
UNITREE_G1_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$UNITREE_G1_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
