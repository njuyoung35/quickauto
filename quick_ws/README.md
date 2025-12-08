# install

```bash
colcon build
. install/setup.bash
ros2 launch ssu2_launch ssu2_launch.launch.xml

ros2 launch autoware_launch autoware.launch.xml
ros2 launch autoware_launch planning_simulator.launch.xml

sudo apt install ros-humble-autoware-map-loader \
ros-humble-autoware-map-projection-loader

sudo apt install ros-humble-generate-parameter-library

# utils
sudo apt install ros-humble-autoware-utils \
ros-humble-autoware-motion-utils \
ros-humble-autoware-vehicle-info-utils

# test
sudo apt install ros-humble-autoware-lint-common \
ros-humble-autoware-testing \
ros-humble-autoware-test-utils \
ros-humble-ros-testing

sudo apt install ros-humble-autoware-interpolation
sudo apt install ros-humble-autoware-osqp-interface

sudo apt install ros-humble-autoware-pose-initializer

sudo apt remove libgoogle-glog-dev libgoogle-glog0v5
wget https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz
tar -xzf v0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake -DWITH_GFLAGS=OFF -DBUILD_TESTING=OFF ..
make -j$(nproc)
sudo make install
```

```
  <depend>autoware_lanelet2_extension</depend>
  <depend>autoware_lanelet2_utils</depend>
  ```

pugixml-dev 이거 autoware_lanelet2_extension 에서 필요