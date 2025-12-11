# 레포 소개

`autowarefoundation/autoware` 메타패키지의 `autoware.repos`라는 `vcs`를 위한 파일에서의 트리 구조를 기반으로 서술했습니다.

## core

- autoware_adapi_msgs
    - adapi는 autoware에서 단순히 차량 뿐만 아니라, V2X, 외부 장치, 외부 네트워크, 외부 애플리케이션 등과 통신하기에 용이하도록 만든 api입니다.
- autoware_cmake
    - cmake 빌드 시스템을 확장한 autoware 전용 빌드 도구 모음입니다.
- autoware_internal_msgs
- autoware_lanelet2_extension
- autoware_msgs
    - planning, control 등 컴포넌트별 사용되는 인터페이스 타입들을 모아놨습니다.
- autoware_rviz_plugins
    - rviz 시각화 도구에서 사용되는 플러그인 모음입니다.
- autoware_utils
- autoware_core ..

### autoware_core

high-quality, stable한 autoware 관련 ROS 패키지들을 저장하는 레포입니다. 정말 핵심 기능만 존재하며, 실질적으로 autoware를 돌릴려면 autoware_universe가 필요합니다.

## universe

### autoware_universe

## launcher

### autoware_launcher

autoware의 메인 엔트리 포인트인 `autoware_launch/launch/autoware.launch.xml`이 담긴 패키지가 들어있습니다.

이외에도 vehicle, sensor 샘플 데이터도 이 레포에 같이 들어있습니다. tf, urdf 등 명세가 담겨 있습니다.

## sensor_component

sensor_component_description 레포를 포함하고 있습니다.

- sensor_component_description (`repo`)
    - camera_description (`package`)
    - imu_description (`package`)
    - velodyne_description (`package`)
    - vls_description (`package`)

센서 명세의 템플릿을 담고 있습니다. 해당 센서가 가질 수 있는 tf, urdf나 parameter 템플릿을 담고 있어서, 실차에 적용시킬 때 이것을 복사해서 calibration을 진행하면 됩니다.

## 기타

- cpp_polyfills
- cudnn_cmake_module
- generate_parameter_library : `param/*.param.yaml` 등과 관련해서 소스코드에 명시적이고 반복적인 `declare_parameter` 등 없이도 코드를 채워주는 codegen 툴입니다.
- Lanelet2 : `lanelet2_core`, `lanelet2_io` 등이 들어있는 레포입니다.
- middleware
- mrm_cmake_modules
- osqp_vendor : QP 솔버 중 하나입니다.
- point_cloud_msg_wrapper
- ros_testing
- RSL (ros supporting library)
- tensorrt_cmake_module
- topic_tools

# 런처의 전파 경로

`planning_simulator.launch.xml`에서 시작하는 런처의 전파 경로를 설명합니다.

간단히 autoware 패키지, 파일, 폴더구조에 대해 설명 드리겠습니다.

autoware_launch(repo)/autoware_launch/launch/autoware.launch.xml는
autoware_launch(repo)/autoware_launch/launch/components에 있는 control, planning 등 컴포넌트급 런치 파일을 include합니다.

그 각각의 컴포넌트들은 autoware_universe(repo)/launch/tier4_*_launch 컴포넌트에 있는 런치 파일을 include합니다.

여기서야 비로소 tier4_*_launch/launch/*.launch.xml 파일들이 각 컴포넌트, map, control, planning, simulator, vehicle 등의 런처를 실행하도록 만듭니다.

그 이후로는 여러 패키지를 한 번에 부르는 거대 런치 파일은 거의 없고, 이 tier4_*_launch 경로 하에서, component container 단위로, 여러 노드와 실행자들을 묶어 실행합니다. 또는 해당 패키지 경로의 `/launch/.launch.xml`을 불러옵니다.

```
- planning_simulator.launch.xml
    - autoware.launch.xml
        - (control)
        - (planning)
        - (map)
        - (vehicle)
    - autoware_launch/launch/components/tier4_simulator_component.launch.xml
        - autoware_simple_planning_simulator
```

## map

- autoware_core (`repo`)
    - map/autoware_map_loader (`package`) : pointcloud 또는 lanelet2 형식의 맵을 로드해서 바이너리 등 압축된 형태로 발행해주는 패키지입니다.
    - map/autoware_map_projection_loader (`package`) : 위도, 경도 기반의 맵을 로드할 때, 프로젝션에 대한 메타데이터도 함께 입력받아서, 카테시안 좌표계로 정규화된 맵이 발행될 수 있도록 돕는 패키지입니다. `map_loader`와 쌍으로 쓰입니다.
- autoware_universe (`repo`)
    - map/autoware_map_tf_generator (`package`) : 차량의 tf와, 맵의 tf을 맞추어 rviz 상에 잘 표시될 수 있도록 중재해주는 패키지입니다.

## control

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

## simulator

- autoware_universe/simulator (`component directory`)
    - autoware_learning_based_vehicle_model (`package`)
        - `autoware_simple_planning_simulator`에서 사용하는 코드적 인터페이스입니다.
    - autoware_simple_planning_simulator (`package`)
        - f1tenth_gym과 유사하게, 한 시간단위 당 차량의 상태를 업데이트 하고, 주변 차량, 신호등 등 또한 같이 업데이트해서 발행하는 핵심 시뮬레이터 노드입니다.

## 흐름 정리

- map : LaneletMapBin 발행.
- planning
    - mission_planning : 서비스를 수신하며, 전역경로를 계산하여 발행합니다. (LaneletRoute 단위)
    - scenario_planning : 지역경로 계획 부분입니다.
        - lane_driving : 기본적인 차선 주행입니다.
            - behavior_path_planning : 차선주행, 회피, 차선변경, 시작, 도착 플래닝 등 경로적으로 기본적인 계획을 수행합니다.
            - behavior_velocity_planner : 상단 컴포넌트에서 구해진 경로를 가지고 가능한 속도 계획을 세웁니다.
            - motion_planner : 더 저수준에서의 동작 계획을 세웁니다. 이후 여러 검증과 path 및 velocity smoother를 거쳐 control에게 궤적을 전달합니다. 저희 프로젝트에서 smoothing은 생략했습니다.
        - parking : 점유격자지도를 이용해, 그리드 기반 경로계획을 새우는 컴포넌트나, 여기서는 채용 안 했습니다.
- control : 궤적에 따라 mpc + pp + pid로 저수준 명령 생성
- simulator : 저수준 명령을 해석해 차량 상태를 업데이트
    - dummy_perception_publisher 및 다른 dummy_publisher들을 통합해서 가상 장애물, scan, 오브젝트, 신호등 정보를 만들어냅니다.
    - 다시 지역경로 생성의 시작점인 behavior_path_planning에게 통신하여, 차량 시뮬레이션이 루프로 돌도록 합니다.
- rviz : initial pose, goal pose를 설정할 때 mission_planning과 통신해 새로운 경로를 만들어냅니다.