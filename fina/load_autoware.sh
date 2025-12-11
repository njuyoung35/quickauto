#!/bin/bash

if [[ "$0" == "$BASH_SOURCE" ]]; then
    echo "This script must be sourced. Use: . $0"
    exit 1
fi

# git clone

git clone -b humble https://github.com/autowarefoundation/autoware.git
cd autoware
rm .git -rf
rm .github -rf

# another git clone (vcs)
cat >> autoware/autoware.repos << 'EOF'
  osqp_vendor:
    type: git
    url: https://github.com/tier4/osqp_vendor.git
    version: main
  ros_testing:
    type: git
    url: https://github.com/ros2/ros_testing.git
    version: humble
  Lanelet2:
    type: git
    url: https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git
    version: master
  mrm_cmake_modules:
    type: git
    url: https://github.com/KIT-MRT/mrt_cmake_modules.git
    version: master
  generate_parameter_library:
    type: git
    url: https://github.com/PickNikRobotics/generate_parameter_library.git
    version: main
  RSL:
    type: git
    url: https://github.com/PickNikRobotics/RSL.git
    version: main
  cpp_polyfills:
    type: git
    url: https://github.com/PickNikRobotics/cpp_polyfills.git
    version: main
  cudnn_cmake_module:
    type: git
    url: https://github.com/tier4/cudnn_cmake_module.git
    version: main
  tensorrt_cmake_module:
    type: git
    url: https://github.com/tier4/tensorrt_cmake_module.git
    version: main
  point_cloud_msg_wrapper:
    type: git
    url: https://gitlab.com/ApexAI/point_cloud_msg_wrapper.git
    version: master
  topic_tools:
    type: git
    url: https://github.com/ros-tooling/topic_tools.git
    version: humble
  grid_map:
    type: git
    url: https://github.com/ANYbotics/grid_map.git
    version: humble
  filters:
    type: git
    url: https://github.com/ros/filters.git
    version: ros2
  sophus:
    type: git
    url: https://github.com/stonier/sophus.git
    version: release/1.3.x
  velodyne:
    type: git
    url: https://github.com/ros-drivers/velodyne.git
    version: ros2
  navigation2:
    type: git
    url: https://github.com/ros-navigation/navigation2.git
    version: humble
  diagnostics:
    type: git
    url: https://github.com/ros/diagnostics.git
    version: ros2-humble
  BehaviorTree.ROS2:
    type: git
    url: https://github.com/BehaviorTree/BehaviorTree.ROS2.git
    version: humble
  behaviortree_cpp_v3-release:
    type: git
    url: https://github.com/BehaviorTree/behaviortree_cpp_v3-release.git
    version: release/humble/behaviortree_cpp_v3
  nmea_msgs:
    type: git
    url: https://github.com/ros-drivers/nmea_msgs.git
    version: ros2
EOF
  # ros_canopen:
  #   type: git
  #   url: https://github.com/ros-industrial/ros_canopen.git
  #   version: melodic-devel
  # catkin:
  #   type: git
  #   url: https://github.com/ros/catkin.git
  #   version: noetic-devel
  # message_generation:
  #   type: git
  #   url: https://github.com/ros/message_generation.git
  #   version: noetic-devel
  # gencpp:
  #   type: git
  #   url: https://github.com/ros/gencpp.git
  #   version: noetic-devel
  # genmsg:
  #   type: git
  #   url: https://github.com/ros/genmsg.git
  #   version: noetic-devel
  # geneus:
  #   type: git
  #   url: https://github.com/jsk-ros-pkg/geneus.git
  #   version: master
  # gennodejs:
  #   type: git
  #   url: https://github.com/RethinkRobotics-opensource/gennodejs.git
  #   version: kinetic-devel
  # genlisp:
  #   type: git
  #   url: https://github.com/ros/genlisp.git
  #   version: noetic-devel
  # genpy:
  #   type: git
  #   url: https://github.com/ros/genpy.git
  #   version: noetic-devel
mkdir src
vcs import src < autoware.repos

sed -i 's|#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>|#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>|g' src/core/autoware_core/common/autoware_trajectory/src/pose.cpp

perl -i -0777 -pe 's{
ament_target_dependencies\(concatenate_data\s+
SYSTEM\s+
autoware_point_types\s+
autoware_sensing_msgs\s+
autoware_utils\s+
autoware_vehicle_msgs\s+
managed_transform_buffer\s+
pcl_conversions\s+
pcl_ros\s+
rclcpp\s+
sensor_msgs\s+
tf2_ros\s*\)
}{
ament_target_dependencies(concatenate_data
  SYSTEM
  autoware_point_types
  autoware_sensing_msgs
  autoware_utils
  autoware_vehicle_msgs
  managed_transform_buffer
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2_ros
  diagnostic_updater
)
}xgs' src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor/CMakeLists.txt

# install dependencies via apt and python
sudo apt-get update
sudo apt install libpugixml-dev \
librange-v3-dev \
ros-humble-magic-enum \
ros-humble-pcl-ros \
ros-humble-proxsuite \
nlohmann-json3-dev \
libcgal-dev \
libcpprest-dev \
unzip

pip install typeguard
pip install gdown