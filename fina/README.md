# fina

## 사용법

### 주의

```bash
. <쉘스크립트>
source <쉘스크립트>
```

위와 같은 형태로 실행해야 하는 스크립트들이 많이 있습니다. `./<쉘스크립트>`와 같이 실행하면 의도적으로 실패를 반환하는 스크립트들이 있으니 주의하고 사용법을 잘 따라주세요.

### git clone fina 최초 후

```bash
. load_autoware.sh
```

1. `autowarefoundation/autoware` 레포를 복붙합니다.
2. `autoware/autoware.repos` 파일에 추가적으로 필요한 의존성을 추가합니다.
    - osqp_vendor
    - ros_testing
    - Lanelet2
    - mrm_cmake_modules
    - generate_parameter_library
    - RSL
    - cpp_polyfills
    - cudnn_cmake_module
    - tensorrt_cmake_module
    - point_cloud_msg_wrapper
3. `autoware/autoware.repos`를 바탕으로 `vcs import src < autoware.repos` 명령어를 실행해 레포를 더 불러옵니다.
4. 기존 소스코드에 문제 있는 부분을 `sed -i`로 수정합니다.
    - `autoware/src/core/autoware_core/common/autoware_trajectory/src/pose.cpp`
5. 추가 의존성을 설치합니다.
    - libpugixml-dev
    - librange-v3-dev
    - ros-humble-magic-enum
    - ros-humble-pcl-ros
    - ros-humble-proxsuite
    - nlohmann-json3-dev
6. 파이썬 관련 추가 의존성을 설치합니다.
    - typeguard

### 빌드

```bash
. build.sh
```

1. `pop.sh`을 실행합니다.
    - `fina_ws`에서 수정한 사항을, `autoware/src`에 반영시켜주는 역할을 합니다.
2. `pkgs.txt`에 있는 패키지들에 대하여 `colcon build --packages-up-to <패키지명>`을 실행합니다.
    - `pkgs.txt`는 단순히 줄바꿈으로 구별된 패키지 이름 문자열의 리스트입니다.
    - 개발 시 부분적 빌드를 위해 사용했으며, 최종 사용 시 `autoware_launch` 하나만 있게 됩니다.

### 실행

```bash
. run.sh
```

`run.sh`의 내용은 대략 이러합니다 (`cd autoware` 가정):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autoware_launch
source autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/misys/sample-map-planning
```

### second terminal

```bash
cd fina/autoware
. opt/ros/humble/setup.bash
. install/setup.bash # autoware 위치에서
# 기본 형식
ros2 topic pub /planning/scenario_planning/scenario autoware_internal_planning_msgs/Scenario "{current_scenario: 'LaneDriving', activating_scenarios: ['LaneDriving']}"
```

### third terminal

```bash
cd fina/autoware
. opt/ros/humble/setup.bash
. install/setup.bash # autoware 위치에서
ros2 topic pub /perception/object_recognition/objects autoware_perception_msgs/msg/PredictedObjects "{header: {stamp: {sec: $(date +%s), nanosec: 0}, frame_id: 'map'}, objects: []}"
```

### fourth terminal

```bash
cd fina/autoware
. opt/ros/humble/setup.bash
. install/setup.bash # autoware 위치에서
ros2 topic pub /system/operation_mode/state autoware_adapi_v1_msgs/msg/OperationModeState "
stamp:
  sec: $(date +%s)
  nanosec: 0
mode: 2  # AUTONOMOUS = 2
is_autoware_control_enabled: true
is_in_transition: false
is_stop_mode_available: true
is_autonomous_mode_available: true
is_local_mode_available: true
is_remote_mode_available: true
"
```

### fifth terminal

```bash
cd fina/autoware
. opt/ros/humble/setup.bash
. install/setup.bash # autoware 위치에서
ros2 topic pub /planning/scenario_planning/parking/costmap_generator/occupancy_grid nav_msgs/msg/OccupancyGrid "
header:
  stamp:
    sec: $(date +%s)
    nanosec: 0
  frame_id: 'map'
info:
  map_load_time:
    sec: $(date +%s)
    nanosec: 0
  resolution: 1.0
  width: 0
  height: 0
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
data: []
"
```

### sixth terminal

```bash
cd fina/autoware
. opt/ros/humble/setup.bash
. install/setup.bash # autoware 위치에서
ros2 topic pub /perception/occupancy_grid_map/map nav_msgs/msg/OccupancyGrid "
header:
  stamp:
    sec: $(date +%s)
    nanosec: 0
  frame_id: 'map'
info:
  map_load_time:
    sec: $(date +%s)
    nanosec: 0
  resolution: 1.0
  width: 0
  height: 0
  origin:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
data: []
"
```

## scripts

### pop.sh

1. `fina_ws/src`에서 수정된 내역이 있다면 확인해서, `autoware/src`에 반영합니다.
2. (기본값) 그리고 `autoware/src`의 내역을 `fina_ws/src`로 복사합니다.
    - 개발 시 필요한 양방향 동기화를 위한 코드입니다.

이 작업들은 `config.yaml`에서 읽은 내용을 바탕으로 진행합니다.

## autoware에 대하여

ROS 기반의 오픈소스 자율주행 소프트웨어 스택입니다. 일본의 나고야 대학, tier IV 등과 관련 되어있습니다.

### Ad API

Ad API라는 노트북, 스마트폰 기기, 조이스틱 등 다양한 기기와 상호작용할 수 있는 표준화된 인터페이스를 제공합니다.

### 컴포넌트 기반 설계

`autoware_core`, `autoware_universe` 등의 레포에서 인지, 플래닝, 컨트롤 등 컴포넌트 단위와, 그 하위 패키지들을 제공해서 부분적으로 개발하고, 교체할 수 있습니다.

### 벤더별 확장

다양한 sensor, vehicle에 적용될 수 있도록 명세와 도구를 제공합니다. Tier IV, AWF 등 다양한 벤더별 솔루션을 제공합니다.

### 데이터

rosbag, 머신러닝, 강화학습 등을 지원하기 위한 도구, 데이터들도 생겨나고 있습니다.

## sample-map-planning

이 데이터는 일본 가시와시 지바현에 위치한 도쿄대학 가시와 캠퍼스 주위의 HDMap (정밀도로지도, 벡터지도) 및 pointcloud 데이터가 담긴, autoware에서 샘플로 제공하는 데이터입니다.

## lanelet2

독일의 FZI, Research Center for Information Technology 연구기관에서 개발한, cpp 기반의 고정밀 지도 데이터 형식 및 라이브러리입니다.

사용되는 파일 형식은 .osm, 그 중 .josm의 확장격 성격을 띠었으며, node와 way, relation으로 구성됩니다.

lanelet2 데이터 형식에 대한 io, 바이너리 인코딩, 쿼리 등을 지원합니다.

autoware에서 사용하는 HDMap의 형식은 lanelet2를 따릅니다. LaneletMapBin이라는 형태로 통신을 하되, lanelet2 라이브러리가 제공하는 쿼리, 함수 등을 이용해 효율적으로 벡터정보를 조회해 플래닝 등 컴포넌트에서 사용할 수 있습니다.

## 의존성에 대하여

`load_autoware.sh`은 `autowarefoundation/autoware`와 그곳의 `autoware.repos`에 의존성을 추가하고, `vcs import`를 통해 수많은 의존성을 로컬로 불러옵니다.

기존 `autowarefoundation/autoware`는 실제 autoware 사용자들이 autoware 관련 레포들과 패키지들을 불러 빌드하는 메타패키지 역할을 합니다.

