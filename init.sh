#!/bin/bash

sudo apt update

# xonsh (python과 shell을 결합한 스크립트 언어)
sudo apt install xonsh

# cpp json
sudo apt install nlohmann-json3-dev

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

# param gen
sudo apt install ros-humble-generate-parameter-library

# planning
sudo apt install ros-humble-autoware-velocity-smoother \
ros-humble-autoware-trajectory \
ros-humble-autoware-objects-of-interest-marker-interface \
ros-humble-autoware-planning-factor-interface \
ros-humble-autoware-object-recognition-utils

# utils
sudo apt install ros-humble-autoware-utils \
ros-humble-autoware-motion-utils \
ros-humble-autoware-vehicle-info-utils

# related to cmake
sudo apt install ros-humble-autoware-cmake \
ros-humble-magic-enum

# test
sudo apt install ros-humble-autoware-lint-common \
ros-humble-autoware-testing \
ros-humble-autoware-test-utils \
ros-humble-ros-testing

sudo apt install ros-humble-autoware-interpolation
sudo apt install ros-humble-autoware-osqp-interface

sudo apt install ros-humble-autoware-pose-initializer