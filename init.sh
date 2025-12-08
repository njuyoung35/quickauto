#!/bin/bash

git clone https://github.com/autowarefoundation/autoware_core.git
git clone https://github.com/autowarefoundation/autoware_universe.git
git clone https://github.com/autowarefoundation/autoware_lanelet2_extension.git
git clone https://github.com/autowarefoundation/autoware_launch.git
git clone https://github.com/autowarefoundation/autoware_msgs.git
git clone https://github.com/autowarefoundation/autoware_utils.git
git clone https://github.com/tier4/sensor_component_description.git
git clone https://github.com/tier4/tier4_autoware_msgs.git

sudo apt update

# autoware_lanelet2_extension 의존성
sudo apt install libpugixml-dev

# glog
sudo apt install libgoogle-glog-dev

# lanelet2 관련
sudo apt install ros-humble-lanelet2 \
ros-humble-lanelet2-core \
ros-humble-lanelet2-io \
ros-humble-lanelet2-projection

# map
sudo apt install ros-humble-autoware-map-loader \
ros-humble-autoware-map-projection-loader

sudo apt install ros-humble-generate-parameter-library

# utils
sudo apt install ros-humble-autoware-utils \
ros-humble-autoware-motion-utils \
ros-humble-autoware-vehicle-info-utils

# autoware_cmake
sudo apt install ros-humble-autoware-cmake

# test
sudo apt install ros-humble-autoware-lint-common \
ros-humble-autoware-testing \
ros-humble-autoware-test-utils \
ros-humble-ros-testing

sudo apt install ros-humble-autoware-interpolation
sudo apt install ros-humble-autoware-osqp-interface

sudo apt install ros-humble-autoware-pose-initializer