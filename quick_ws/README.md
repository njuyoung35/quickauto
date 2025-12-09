# quick_ws

## 스택 구조

메인 엔트리 포인트인 `autoware_launch`의 호출 흐름, 그리고 기능적 구분으로 목차를 나눠 설명하겠습니다.

가시성을 위해, 많은 패키지가 포함되어 있는 경로를 `directory`로 표기하고, 그 중 autoware가 component라고 명칭하는 단위에 대해서는 `component directory`라는 표기를 사용합니다.

레포 단위에 대해서는 `repo`, 패키지 단위에 대해서는 `package`, 패키지 단위이지만, `init.sh`에서 apt로 설치한 단위에 대해서는 `apt-package`로 표기합니다.

표기가 없는 최상위 구분단위는, 레포와 경로를 넘어선, 의미론적 그룹 단위입니다.

- launch
    - autoware_launch (`repo`)
        - autoware_launch (`package`)
        - vehicle/sample_vehicle_launch (`directory`)
        - sensor_kit/sample_sensor_kit_launch (`directory`)
    - autoware_universe (`repo`)
        - launch (`directory`)
- map
    - autoware_core (`repo`)
        - map/autoware_map_loader (`apt-package`)
        - map/autoware_map_projection_loader (`apt-package`)
    - autoware_universe (`repo`)
        - map/autoware_map_tf_generator (`package`)
- lanelet2
    - autoware_lanelet2_extension (`repo`)
    - autoware_core (`repo`)
        - common/autoware_lanelet2_utils (`package`)
- planning
    - autoware_core (`repo`)
        - planning (`component directory`)
            - behavior_path_planner (`directory`)
            - behavior_velocity_planner (`directory`)
    - autoware_universe (`repo`)
        - planning (`component directory`)
            - behavior_path_planner (`directory`)
            - behavior_velocity_planner (`directory`)
- control
    - autoware_universe (`repo`)
        - control (`component directory`)
- simulator
    - autoware_universe (`repo`)
        - simulator (`component directory`)
            - autoware_learning_based_vehicle_model (`package`)
            - autoware_simple_planning_simulator (`package`)
- sensor
    - sensor_component_description (`repo`)
- msgs
    - autoware_msgs (`repo`)
    - tier4_autoware_msgs (`repo`)
- test
- devtools
- utils
- etc

### launch

### map

- autoware_core (`repo`)
    - map/autoware_map_loader (`apt-package`) : pointcloud 또는 lanelet2 형식의 맵을 로드해서 바이너리 등 압축된 형태로 발행해주는 패키지입니다.
    - map/autoware_map_projection_loader (`apt-package`) : 위도, 경도 기반의 맵을 로드할 때, 프로젝션에 대한 메타데이터도 함께 입력받아서, 카테시안 좌표계로 정규화된 맵이 발행될 수 있도록 돕는 패키지입니다. `map_loader`와 쌍으로 쓰입니다.
- autoware_universe (`repo`)
    - map/autoware_map_tf_generator (`package`) : 차량의 tf와, 맵의 tf을 맞추어 rviz 상에 잘 표시될 수 있도록 중재해주는 패키지입니다.

### lanelet2

```bash
# lanelet2 관련
# cpp 라이브러리 lanelet2 레포에 있는 패키지를 불러옵니다.
sudo apt install ros-humble-lanelet2 \
ros-humble-lanelet2-core \
ros-humble-lanelet2-io \
ros-humble-lanelet2-projection

# autoware_lanelet2_extension의 의존성
sudo apt install libpugixml-dev

# autoware-lanelet2-utils (버전 1.4.0) 제거 (충돌 발생 위험 있음, 1.5.0 필요)
# 빌드 도중 버전 충돌이 심각하게 낫던 패키지로, 혹여나 미리 설치되어 있을 것을 방지하기 위해 삭제 커맨드를 넣어놨습니다.
# apt가 아니라, 레포에서 다운 후 빌드해야 최신버전의 autoware-lanelet2-utils를 빌드할 수 있습니다.
sudo apt remove ros-humble-autoware-trajectory \
ros-humble-autoware-route-handler \
ros-humble-autoware-lanelet2-utils
```

### planning

`planning.md`에서 따로 다룹니다.

### control

- autoware_universe/control (`component directory`)
    - autoware_mpc_lateral_controller (`package`)
        - mpc 제어기입니다.
    - autoware_pid_longitudinal_controller (`package`)
        - pid 제어기입니다.
    - autoware_pure_pursuit (`package`)
        - pure pursuit 제어기입니다.
    - autoware_trajectory_follower_base (`package`)
        - `autoware_trajectory_follower_node`가 알고리즘적으로 참조하는 패키지입니다. 여기서 mpc, pure pursuit, pid 등 제어기를 선택해서 계산을 수행합니다.
    - autoware_trajectory_follower_node (`package`)
        - 궤적을 입력받아, 아커만 컨트롤 신호를 발행하는 핵심 노드입니다.

### simulator

- autoware_universe/simulator (`component directory`)
    - autoware_learning_based_vehicle_model (`package`)
        - `autoware_simple_planning_simulator`에서 사용하는 코드적 인터페이스입니다.
    - autoware_simple_planning_simulator (`package`)
        - f1tenth_gym과 유사하게, 한 시간단위 당 차량의 상태를 업데이트 하고, 주변 차량, 신호등 등 또한 같이 업데이트해서 발행하는 핵심 시뮬레이터 노드입니다.

### sensor

- sensor_component_description (`repo`)
    - camera_description (`package`)
    - imu_description (`package`)
    - velodyne_description (`package`)
    - vls_description (`package`)

`quickauto/README.md`에서 설명한 것처럼, 센서 명세를 담고 있습니다. 시뮬레이터라서 센서가 필요 없을 것 처럼 보이더라도, 센서의 tf를 발행을 하고 있으면, 그것에 따라 보정된 데이터를 차량이 인지하는 효과를 내야할 수 있다고 판단해 포함시켰습니다.

### msgs

#### autoware_msgs (`repo`)

- autoware_control_msgs (`package`)
    - `Lateral`, `Longitudinal`, `Control` 같은 저수준 제어 명령 인터페이스가 담겨있습니다.
- autoware_system_msgs (`package`)
- autoware_vehicle_msgs (`package`)
- autoware_planning_msgs (`package`)
- autoware_map_msgs (`package`)

#### tier4_autoware_msgs (`repo`)

- `tier4_control_msgs` (`package`)
- `tier4_vehicle_msgs` (`package`)
- `tier4_planning_msgs` (`package`)
- `tier4_perception_msgs` (`package`)
    - 시뮬레이터가 가상으로 인지 데이터를 보내기 위해서라도, 인지 인터페이스가 필요합니다.
- `tier4_rtc_msgs` (`package`)
    - RTC(request to cooperate), V2X와 관련해 다른 차량과 통신하기 위해 쓰이는 인터페이스를 담은 패키지입니다.

### test

빌드에 방해되는 테스트 관련 내용들은 최대한 제거했지만, 그럼에도 공통적으로 널리 쓰이는 패키지는 빌드 편의상 포함시켰습니다.

- autoware_universe (`repo`)
    -  common/autoware_fake_test_node (`package`)

```bash
sudo apt install ros-humble-autoware-lint-common \
ros-humble-autoware-testing \
ros-humble-autoware-test-utils \
ros-humble-ros-testing
```

### devtools

```bash
sudo apt install xonsh
```

쉘과 파이썬을 결합한 xonsh라는 스크립트 언어입니다. `repos.yaml`을 파이썬의 `yaml` 패키지를 통해 손쉽게 읽어들이고 파일 이동 및 복사 등 간단한 작업을 수행하기 위해 추가한 의존성입니다.

```bash
sudo apt install ros-humble-autoware-cmake \
ros-humble-magic-enum \
ros-humble-autoware-pyplot \
ros-humble-generate-parameter-library
```

- `autoware-cmake` : autoware에서 사용하는 cmake 패키지입니다.
- `generate-parameter-library` : 패키지의 `/param/*.param.yaml` 파일과 연결시켜서, cpp코드에 `declare_parameter` 없이 자동으로 코드를 생성해서 등록 및 초기화 코드를 채워주는 도구입니다.

### utils

각종 이름에 'utils'가 붙은 패키지들입니다.

```bash
sudo apt install ros-humble-autoware-utils \
ros-humble-autoware-motion-utils \
ros-humble-autoware-vehicle-info-utils
```

### etc

- `autoware-interpolation` : 보간을 위한 패키지입니다.
- `autoware-osqp-interface` : osqp라는 qp솔버를 사용하기 쉽게 래핑해놓은 패키지입니다.

```bash
sudo apt install ros-humble-autoware-interpolation \
ros-humble-autoware-osqp-interface \
ros-humble-autoware-pose-initializer
# cpp json
sudo apt install nlohmann-json3-dev
# glog
sudo apt install libgoogle-glog-dev
```

## 스크립트

### run.sh

`process.xsh`와 `source /opt/ros/humble/setup.bash`, `colcon build`, `source install/setup.bash`를 연달아 실행한 후 `autoware_launch`를 실행하는 스크립트입니다.

#### 사용법

```bash
./run.sh
```

### process.xsh

`repos.yaml`에 명세된 대로, `quickauto` 경로의 레포에서 부분경로를 복사해서 src의 부분경로에 복사합니다. 이미 말단 디렉토리 경로가 존재하면, 스킵합니다.

기존 레포들을 전부 build하려면 굉장히 많은 시간이 소요되기에, 부분적으로, 시뮬레이터에 필요한 부분만 가져오도록 하는 전략을 취했습니다.

#### 사용법

```bash
xonsh process.xsh
```

### del.sh (개발용)

`autoware_launch`, `autoware_universe/launch` 등 일부 경로를 제외하고 전부 초기화하며, 모든 `/build`, `/install`, `/log`를 삭제합니다.

또한 빌드 실패 시 부산물로 생기는 `core.*`도 같이 제거합니다.

#### 사용법

```bash
./del.sh
```

### reset_package.sh (개발용)

`/build`와 `/install`에서 주어진 패키지를 지웁니다.

#### 사용법

```bash
./reset_package.sh <패키지명>
```