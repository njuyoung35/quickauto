#!/bin/bash

if [[ "$0" == "$BASH_SOURCE" ]]; then
    echo "This script must be sourced. Use: . $0"
    exit 1
fi

. pop.sh

cd autoware

source /opt/ros/humble/setup.bash
colcon build --packages-select autoware_launch
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/misys/fina/sample-map-planning

cd -