#!/bin/bash

xonsh process.xsh

rm src/autoware_universe/control/autoware_pid_longitudinal_controller/autoware_pid_longitudinal_controller/test -rf
sed -i '/^if(BUILD_TESTING)/,/^endif()/d' src/autoware_universe/control/autoware_pid_longitudinal_controller/autoware_pid_longitudinal_controller/CMakeLists.txt
rm src/autoware_universe/control/autoware_mpc_lateral_controller/autoware_mpc_lateral_controller/test -rf
sed -i '/^if(BUILD_TESTING)/,/^endif()/d' src/autoware_universe/control/autoware_mpc_lateral_controller/autoware_mpc_lateral_controller/CMakeLists.txt

rm build/autoware_launch -rf
rm install/autoware_launch -rf

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml map_path:=/home/misys/quickauto/sample-map-planning