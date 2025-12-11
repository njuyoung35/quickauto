#!/bin/bash

# Make sure to run this in the workspace directory
mkdir src
git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust

# Make sure to run this in the workspace directory
vcs import src < src/ros2_rust/ros2_rust_humble.repos

# 풀리퀘스트 성공으로 인한 주석화. fix: update rosidl_runtime_rs dependency version to 0.5 #11
# # ros2-rust/rosidl_rust/rosidl_generator_rs/resource/Cargo.toml.em에 rosidl_runtime_rs에 대한 의존성 버전 문제가 있어서 수정해야 합니다!
# cd ros2-rust/rosidl_rust/rosidl_generator_rs/resource
# find . -name "Cargo.toml.em" -exec sed -i 's/rosidl_runtime_rs = "0\.4"/rosidl_runtime_rs = "0.5"/g' {} \;
# cd -

# 풀리퀘스트 성공으로 인한 주석화. chore: bump rclrs dependency version to 0.6 and rosidl_runtime_rs to 0.5 #14 
# # ros2-rust/examples/rclrs 아래의 패키지 파일들의 Cargo.toml 의존성 버전 문제가 있어서 수정해야 합니다!
# cd ros2-rust/examples/rclrs
# find . -name "Cargo.toml" -exec sed -i 's/rclrs = "0\.5"/rclrs = "0.6"/g' {} \;
# find . -name "Cargo.toml" -exec sed -i 's/rosidl_runtime_rs = "0\.4"/rosidl_runtime_rs = "0.5"/g' {} \;
# cd -

# You can also do `source /opt/ros/humble/setup.bash` in bash
. /opt/ros/humble/setup.sh

colcon build