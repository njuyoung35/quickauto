#!/bin/bash

ros2 pkg create $1 \
    --build-type ament_cmake \
    --dependencies rclrs $@ \
    --description "My Rust node package" \
    --license "MIT"

cd $1
sed -i 's/ament_cmake/ament_cargo/g' package.xml
sed -i "/<test_depend>ament_lint_auto</test_depend>/d" package.xml
sed -i "/<test_depend>ament_lint_common</test_depend>/d" package.xml

cargo init --edition 2021
cargo add rclrs@0.6.0
for dep in "$@"; do
    echo "$dep = \"*\"" >> Cargo.toml
done