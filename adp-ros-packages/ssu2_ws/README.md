# ssu2_ws

## how to start

```bash
cd ~/forza_ws/racestack (지도만 띄울때는 필요없지만 앞으로 planner랑 같이 사용할떄는 필수인 듯 해요)
source install/setup.bash 
cd ~/ssu2_ws
source install/setup.bash

ros2 launch ssu2_launch ssu2_launch.launch.xml
==> 결과
벡터지도 Rviz2에 뜨고,

misys@leafy:~/ssu2_ws$ ros2 node list
/launch_ros_43
/map/lanelet2_map_loader
/map/lanelet2_map_visualization
/map/map_container
/map/map_hash_generator
/map/map_projection_loader
/map/pointcloud_map_loader
/map/vector_map_tf_generator
/rviz2
/transform_listener_impl_607894f102c0

misys@leafy:~/ssu2_ws$ ros2 topic list
/api/autoware/get/map/info/hash
/clicked_point
/goal_pose
/initialpose
/map/map_projector_info <<< 얘
/map/pointcloud_map     <<< 얘
/map/vector_map         <<< 얘
/map/vector_map_marker  <<< 얘네가 특히 planner한테 중요한 토픽..??
/parameter_events
/rosout
/tf
/tf_static

```

## colcon build (다양한 옵션들, 복붙용)

```bash
colcon build --symlink-install  <<이거 하면(지금 돼있음. 그냥 참고자료임.)

| 패키지                              | 역할                           | 상태             |
| -------------------------------- | ------------------------------- | --------------- |
| autoware_map_projection_loader   | lat/lon → local xyz 변환         | ✔ 성공           |
| autoware_map_loader              | pointcloud + lanelet2 map 로딩   | ✔ 성공           |
| autoware_lanelet2_extension      | OSM → lanelet2 확장 유틸          | ✔ 성공(경고만 있음) |
| autoware_map_tf_generator        | map frame 구조 TF 생성            | ✔ 성공           |
| autoware_lanelet2_map_visualizer | /map/vector_map_marker 시각화    | ✔ 성공            |
| tier4_map_launch                 | 전체 map pipeline launch         | ✔ 성공           |


Starting >>> autoware_common_msgs
Starting >>> autoware_map_msgs
Starting >>> ssu2_map_msgs
Starting >>> ssu2_map_test
Starting >>> ssu_global_planner
Finished <<< ssu2_map_test [0.15s]                                        
Finished <<< ssu_global_planner [0.21s]                                       
Finished <<< autoware_common_msgs [0.50s]                                      
Starting >>> autoware_planning_msgs
Starting >>> tier4_external_api_msgs
Finished <<< autoware_map_msgs [0.53s]
Finished <<< autoware_planning_msgs [0.49s]                        
Starting >>> autoware_lanelet2_extension                           
Starting >>> autoware_component_interface_specs
Finished <<< tier4_external_api_msgs [0.67s]                       
Finished <<< ssu2_map_msgs [1.48s]                                      
Finished <<< autoware_component_interface_specs [0.69s]
--- stderr: autoware_lanelet2_extension                             
In this package, headers install destination is set to `include` by ament_auto_package. It is recommended to install `include/autoware_lanelet2_extension` instead and will be the default behavior of ament_auto_package from ROS 2 Kilted Kaiju. On distributions before Kilted, ament_auto_package behaves the same way when you use USE_SCOPED_HEADER_INSTALL_DIR option.
---
Finished <<< autoware_lanelet2_extension [1.11s]
Starting >>> autoware_geography_utils
Starting >>> autoware_map_projection_loader
Starting >>> autoware_map_tf_generator
Finished <<< autoware_map_tf_generator [1.93s]                                
Finished <<< autoware_geography_utils [2.45s]
Starting >>> autoware_map_loader
Finished <<< autoware_map_projection_loader [11.8s]
Finished <<< autoware_map_loader [34.5s]                         
Starting >>> autoware_lanelet2_map_visualizer
Starting >>> ssu2_launch
Starting >>> tier4_map_launch
Finished <<< ssu2_launch [0.45s]        
Finished <<< tier4_map_launch [0.46s]   
Finished <<< autoware_lanelet2_map_visualizer [3.70s]                       

Summary: 16 packages finished [43.0s]
  1 package had stderr output: autoware_lanelet2_extension
이렇게 나오면 됨! 근데 이제부터는 새로 생기는 패키지만 따로 colcon build --packages-select XXX  << 이렇게 추가만 하면 되요~!

<!-- colcon build --base-paths src ssu2_msgs --packages-skip-build-finished --symlink-install
     colcon build --base-paths src ssu2_msgs --symlink-install -->
```

## dependencies!
이 부분은 
* ssu2_ws/scripts/process_universe.xsh
* ssu2_ws/repos_universe.yaml을 참고하면 도움이 될 것 같아요.
* ssu2_ws/src/launch/ssu2_launch/launch/ssu2_launch.launch.xml의 코드 5번줄 :  arg name="workspace_path" default="/home/misys/ssu2_ws"
이 부분을 보시면 저는 adp-ros-packages 디렉을 지우고 ssu2_ws만 빼서 작업했어서 해당경로가 없어요. 이 default경로를 default="/home/misys/adp-ros-packages/ssu2_ws" 로 바꿔사용가능해요! 

```bash
sudo apt update
sudo apt install libpugixml-dev

# rust setup
sudo apt install -y git libclang-dev python3-pip python3-vcstool
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git

pip install trimesh

sudo apt install ros-humble-autoware-cmake \
ros-humble-ros-testing

sudo apt install ros-humble-autoware-lanelet2-extension \
ros-humble-autoware-component-interface-specs

# lanelet2 setup
sudo apt install ros-humble-lanelet2 \
ros-humble-lanelet2-core \
ros-humble-lanelet2-io \
ros-humble-lanelet2-projection
```

저는 일단 제 환경에서 돌아가게 되어버려서 처음부터 정확한 의존성 경로를 다 체크한 건 아니니, 부족한 것 있으면 채워주세요~

## 노드가 죽지 않았을 때..?

- .launch.xml로 한 번에 많은 노드들을 실행하고, map_loader처럼 qos 정책이나 내부 로직 상 장수하는 노드들이 있는 걸로 보이는데.. `ros2 node list`에 중복되어 살아남는게 간혹 있더라고요.
- `ros2 daemon stop` 같은 것보다도 `ps aux`로 조사하고 필터링해서 `kill`하는 게 효과가 있었습니다.

```bash
# ros2 daemon stop
# pkill -f 'ros2\|rclpy\|rclcpp'
# sudo pkill -f ros2
ps aux | grep "map" | grep -v grep | awk '{print $2}' | xargs kill -9
```
