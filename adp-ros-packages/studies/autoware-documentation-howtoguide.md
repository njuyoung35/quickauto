# autoware how to guide

## 1. creating autoware repo

- Meta-repository : `autoware.repos`를 포함, 중앙집중화된 참조, 구성, 버저닝, 관리
- autowarefoundation/autoware 레포를 fork하기
- `sample_sensor_kit`, `sample_vehicle_launch`, `autoware_individual_params`, `autoware_launch` (서브)레포 또한 포크하고 수정해야 한다.
- `src` 디렉토리 만들고에 `vcs import src < autoware.repos`로 저장소 복제
- `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`

### 주요 복사 레포들

- core/autoware_msgs
- core/autoware_adapi_msgs
- core/autoware_internal_msgs
- core/autoware_cmake
- core/autoware_utils
- core/autoware_lanelet2_extension
- core/autoware_core
- core/autoware_rviz_plugins
- universe/autoware_universe
- universe/external/..
- launcher/autoware_launch
- sensor_component/external/..
- sensor_component/transport_drivers
- sensor_component/ros2_socketcan

## 2. creating vehicle and sensor model

### creating sensor model

https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/creating-sensor-model/#sensor_kit_calibrationyaml

- sensor model
    - calibration (transformation), launch files 등 포함하고 있음
    - 센서 모델 (sensor kit)는 세 패키지로 구성됨.

```
src/
    sensor_kit/
        sample_sensor_kit_launch/
            common_sensor_launch/
            sample_sensor_kit_description/
                config/
                    sensor_kit_calibration.yaml # 센서 간 상대위치/방향 정의
                    sensors_calibration.yaml    # 센서 키트와 차량 base_link 간 변환 정의
                urdf/                           # 이 둘을 토대로, .xacro 차량 프레임에 센서 연결해주자
                    sensor_kit.xacro
                    sensors.xacro
            sample_sensor_kit_launch/
```

```yaml
# sensor_kit_calibration.yaml
sensor_kit_base_link:
  # euler foramt [x, y, z, roll, pitch, yaw]를 명세해주자.
  # calibration step을 밟기 전 까지는 모두 0.0으로 설정해주자.
  velodyne_top_base_link:
    x: 0.000000
    y: 0.000000
    z: 0.000000
    roll: 0.000000
    pitch: 0.000000
    yaw: 0.000000
  camera0/camera_link:
    x: 0.000000
    y: 0.000000
    z: 0.000000
    roll: 0.000000
    pitch: 0.000000
    yaw: 0.000000
  # 여기에 camera, lidar, radar, gnss/ins 등 센서들이 병렬로 추가된다고 보면 된다.
```

```yaml
# sensors_calibration.yaml
# 차량의 base_link 기준으로 위에서 명세한 sensor_kit 좌표계를 차량과 연동시켜주자.
base_link:
  sensor_kit_base_link:
    x: 0.000000
    y: 0.000000
    z: 0.000000
    roll: 0.000000
    pitch: 0.000000
    yaw: 0.000000
```

```xml
<!-- sensor_kit.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="sensor_kit_macro" params="parent x y z roll pitch yaw">
    <xacro:include filename="$(find velodyne_description)/urdf/VLP-16.urdf.xacro"/>
    <xacro:include filename="$(find vls_description)/urdf/VLS-128.urdf.xacro"/>
    <xacro:include filename="$(find camera_description)/urdf/monocular_camera.xacro"/>
    <xacro:include filename="$(find imu_description)/urdf/imu.xacro"/>

    <xacro:arg name="gpu" default="false"/>
    <xacro:arg name="config_dir" default="$(find tutorial_vehicle_sensor_kit_description)/config"/>

    <xacro:property name="sensor_kit_base_link" default="sensor_kit_base_link"/>

    <joint name="${sensor_kit_base_link}_joint" type="fixed">
      <origin rpy="${roll} ${pitch} ${yaw}" xyz="${x} ${y} ${z}"/>
      <parent link="${parent}"/>
      <child link="${sensor_kit_base_link}"/>
    </joint>
    <link name="${sensor_kit_base_link}">
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </link>

    <!-- sensor -->
    <xacro:property name="calibration" value="${xacro.load_yaml('$(arg config_dir)/sensor_kit_calibration.yaml')}"/>

    <!-- lidar -->
    <xacro:VLS-128 parent="sensor_kit_base_link" name="rs_helios_top" topic="/points_raw" hz="10" samples="220" gpu="$(arg gpu)">
      <origin
        xyz="${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['x']}
             ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['y']}
             ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['z']}"
        rpy="${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['roll']}
             ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['pitch']}
             ${calibration['sensor_kit_base_link']['rs_helios_top_base_link']['yaw']}"
      />
    </xacro:VLS-128>

    <!-- camera, gnss 등 센서도 병렬로 추가해나가면 된다 -->
```

- 패키지 갈아끼우면서 수정해야 하는 것들 : `package.xml`에서 이름, `CMakeList.txt`에서 `project( .. )`
- 캘리브레이션 준비: 초기에는 0값으로 설정 후 실제 캘리브레이션 수행

- 여기까지 sample_sensor_kit_description/ 준비는 되었고, 이제 sensor_kit_launch 패키지를 작성해보자
    - 센서와 드라이버(=bridge)를 런치하는 파이프라인 작성하기
    - common_sensor_launch도 활용해서 lidar sensing pipeline 구체적인걸 작성할 수도 있다.
- lidar, imu, camera, GNSS로부터 rectified pointcloud, imu messages, rectified image, gnss/ins messages를 받아올 수 있도록 하자

```
<YOUR-VEHICLE-NAME>_sensor_kit_launch/
      ├─ config/
      ├─ data/
      └─ launch/
+           ├─ camera.launch.xml
+           ├─ gnss.launch.xml
+           ├─ imu.launch.xml
+           ├─ lidar.launch.xml
+           ├─ pointcloud_preprocessor.launch.py
+           └─ sensing.launch.xml # 얘가 나머지 4개 런처 런치
```

```xml
<!-- lidar.launch.xml -->
...
    <group>
      <push-ros-namespace namespace="<YOUR-SENSOR-NAMESPACE>"/>
      <include file="$(find-pkg-share common_sensor_launch)/launch/robosense_Bpearl.launch.xml">
        <arg name="max_range" value="30.0"/>
        <arg name="sensor_frame" value="<YOUR-ROBOSENSE-SENSOR-FRAME>"/>
        <arg name="sensor_ip" value="<YOUR-ROBOSENSE-SENSOR-IP>"/>
        <arg name="host_ip" value="$(var host_ip)"/>
        <arg name="data_port" value="<YOUR-ROBOSENSE-SENSOR-DATA-PORT>"/>
        <arg name="gnss_port" value="<YOUR-ROBOSENSE-SENSOR-GNSS-PORT>"/>
        <arg name="scan_phase" value="0.0"/>
        <arg name="launch_driver" value="$(var launch_driver)"/>
        <arg name="vehicle_mirror_param_file" value="$(var vehicle_mirror_param_file)"/>
        <arg name="container_name" value="pointcloud_container"/>
      </include>
    </group>
```

### creating individual params

다른 차량마다 커스텀화된 센서 calibrations을 사용할 수 있게 해준다. 심지어 같은 런치 파일이더라도

```
individual_params/
└─ config/
     ├─ default/
     │   └─ <YOUR_SENSOR_KIT>/                  # example1
     │        ├─ imu_corrector.param.yaml
     │        ├─ sensor_kit_calibration.yaml
     │        └─ sensors_calibration.yaml
+    ├─ VEHICLE_1/
+    │   └─ <YOUR_SENSOR_KIT>/                  # example2
+    │        ├─ imu_corrector.param.yaml
+    │        ├─ sensor_kit_calibration.yaml
+    │        └─ sensors_calibration.yaml
+    └─ VEHICLE_2/
+         └─ <YOUR_SENSOR_KIT>/                  # example3
+              ├─ imu_corrector.param.yaml
+              ├─ sensor_kit_calibration.yaml
+              └─ sensors_calibration.yaml
```

```bash
# example2 (set vehicle_id as VEHICLE_1)
$ ros2 launch autoware_launch autoware.launch.xml sensor_model:=<YOUR-VEHICLE-NAME>_sensor_kit vehicle_model:=<YOUR-VEHICLE-NAME>_vehicle vehicle_id:=VEHICLE_1
```

### creating vehicle model

```
<YOUR-OWN-AUTOWARE-DIR>/
  └─ src/
       └─ vehicle/
            └─ <YOUR-VEHICLE-NAME>_vehicle_launch/
                 ├─ <YOUR-VEHICLE-NAME>_vehicle_description/
                 └─ <YOUR-VEHICLE-NAME>_vehicle_launch/
```

```
# 이 패키지의 목적 : vehicle dimensions, 3d model of the vehicle 등등 정의하려고
<YOUR-VEHICLE-NAME>_vehicle_description/
   ├─ config/
   │     ├─ mirror.param.yaml
   │     ├─ simulator_model.param.yaml
   │     └─ vehicle_info.param.yaml
   ├─ mesh/
   │     ├─ <YOUR-VEHICLE-MESH-FILE>.dae (or .fbx)
   │     ├─ ...
   └─ urdf/
         └─ vehicle.xacro
```

```yaml
# mirror.param.yaml
# 백미러가 있다면, 그걸 3차원 관점에서 길이가 어떤지, base_link 기준으로 서술하면 된다.
# 없으면 모두 0.0
# lidar PointCloudPreprocessor가 CropBox filter를 사용할 때 필요한 것으로 보임
/**:
  ros__parameters:
    min_longitudinal_offset: 0.0
    max_longitudinal_offset: 0.0
    min_lateral_offset: 0.0
    max_lateral_offset: 0.0
    min_height_offset: 0.0
    max_height_offset: 0.0
```

```yaml
# simulator_model.param.yaml
/**:
  ros__parameters:
    simulated_frame_id: "base_link" # center of the rear axle.
    origin_frame_id: "map"
    vehicle_model_type: "DELAY_STEER_ACC_GEARED" # options: IDEAL_STEER_VEL / IDEAL_STEER_ACC / IDEAL_STEER_ACC_GEARED / DELAY_STEER_ACC / DELAY_STEER_ACC_GEARED
    initialize_source: "INITIAL_POSE_TOPIC" #  options: ORIGIN / INITIAL_POSE_TOPIC
    timer_sampling_time_ms: 25
    add_measurement_noise: False # the Gaussian noise is added to the simulated results
    vel_lim: 50.0 # limit of velocity
    vel_rate_lim: 7.0 # limit of acceleration
    steer_lim: 1.0 # limit of steering angle
    steer_rate_lim: 5.0 # limit of steering angle change rate
    acc_time_delay: 0.1 # dead time for the acceleration input
    acc_time_constant: 0.1 # time constant of the 1st-order acceleration dynamics
    steer_time_delay: 0.24 # dead time for the steering input
    steer_time_constant: 0.27 # time constant of the 1st-order steering dynamics
    x_stddev: 0.0001 # x standard deviation for dummy covariance in map coordinate
    y_stddev: 0.0001 # y standard deviation for dummy covariance in map coordinate
```

```yaml
# vehicle_info.param.yaml
/**:
  ros__parameters:
    wheel_radius: 0.383 # The radius of the wheel, primarily used for dead reckoning.
    wheel_width: 0.235 # The lateral width of a wheel tire, primarily used for dead reckoning.
    wheel_base: 2.79 # between front wheel center and rear wheel center
    wheel_tread: 1.64 # between left wheel center and right wheel center
    front_overhang: 1.0 # between front wheel center and vehicle front
    rear_overhang: 1.1 # between rear wheel center and vehicle rear
    left_overhang: 0.128 # between left wheel center and vehicle left
    right_overhang: 0.128 # between right wheel center and vehicle right
    vehicle_height: 2.5
    max_steer_angle: 0.70 # [rad]
```

.fbx, .dae 포맷 파일로 자동차의 외관을 묘사하고, 추후 vehicle.xacro에서 이를 사용할 수 있다.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- load parameter -->
- <xacro:property name="vehicle_info" value="${xacro.load_yaml('$(find sample_vehicle_description)/config/vehicle_info.param.yaml')}"/>
+ <xacro:property name="vehicle_info" value="${xacro.load_yaml('$(find <YOUR-VEHICLE-NAME>_vehicle_description)/config/vehicle_info.param.yaml')}"/>

  <!-- vehicle body -->
  <link name="base_link">
    <visual>
      <origin xyz="${vehicle_info['/**']['ros__parameters']['wheel_base']/2.0} 0 0" rpy="${pi/2.0} 0 ${pi}"/>
      <geometry>
-       <mesh filename="package://sample_vehicle_description/mesh/lexus.dae" scale="1 1 1"/>
+       <mesh filename="package://<YOUR-VEHICLE-NAME>_vehicle_description/mesh/<YOUR-3D-MESH-FILE>" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

- vehicle interface 관련 launch도 구성하기
- 이제, sensor_model, individual_parameters, vehicle_model을 작성했으니, 이제 시뮬레이터 위에 당신의 차량을 올릴 수 있다.

```
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/Files/autoware_map/sample-map-planning/ vehicle_model:=<YOUR-VEHICLE-MODEL> sensor_model:=<YOUR-SENSOR-KIT> vehicle_id:=<YOUR-VEHICLE-ID>
```

### calibrating sensors

TIER IV의 캘리브레이션 도구를 소개한다.

autoware는 차량에 여러 개의 센서가 부착되어 있을 것으로 기대한다.

- 두 가지 접근 방식:
    - 실제 측정 기반
        - '물리'로 재는 것 외에도, 주행 데이터를 기반에서 역산할 수 있는 알고리즘 도구들이 존재하는 것으로 보임.
        - 그래서 lidar, camera, gnss 등 각 센서 특징에 맞는 도구가 있는 걸로 보임.
        - lidar의 경우 pcd 맵 + NDT 같은걸로.. 적당히 도출 하는 듯..
        - 카메라의 경우, 인쇄된 7x7 체커보드 가지고 막 인식하면서 캘리브레이션 하는 동영상이 보입니다..
        - 센서 하나씩 하는 법 외에도, 센서 두 개를 같이 돌려서 fusion해서 캘리브레이션할 수 있는 방법도 있는 걸로 보여요.
    - CAD 모델 기반

## 3. creating vehicle interface

- autoware 노드들이 열심히 구한 control message를, 실제 당신의 차량이 이해하는 형식으로 바꿔 전달하자.
    - CAN, serial message 같은 형태로..
    - lateral controls : steering tire angle, steering tire rotation rate
    - longitudinal controls : speed, acceleration, jerk
- 또한 반대로, 차량이 발산하는 데이터 형식을 디코드하여 ros2 topics 형식으로 만들어 publish하자.

- 이것들로 부터 입력을 받는다:
    - DiagnosticsManager
    - Other Controls : emergency, external, joystick, ..
    - Vehicle Control Command (autoware_auto_vehicle_msgs/AckermannControlCommand from Control)
    - Vehicle Sign Commands(autoware_auto_vehicle_msgs/)
        - HandBrakeCommand
        - HazardLightsCommand
        - HeadLightsCommand
        - HornCommand
        - StationaryLockingCommand
        - TurnIndicatorsCommand
        - WipersCommand
- Vehicle Interface!
- 이것들에게 출력하거나, 상호작용하기도 한다:
    - ActuationCommand : acceleration, brake, steering
    - Vehicle Communication : Vehicle Specific Protocol
    - Vehicle Signal Reports (autoware_auto_vehicle_msgs/)
        - GearReport
        - HandBrakeReport
        - HazardLightsReport
        - HeadLightsReport
        - HornReport
        - TurnIndicatorsReport
        - WipersReport
    - Steering Status : steering_angle
        - Control 컴포넌트로 역으로 publish하는 역전파 같은 통신이 존재하는 걸로 보인다.
    - Actuation Status : acceleration, brake, steering
        - Control 컴포넌트로 역으로 publish하는 역전파 같은 통신이 존재하는 걸로 보인다.
    - Vehicle Odometry : geometry_msgs/TwistWithCovarianceStamped
        -> 측위가 데드 레코닝 때 사용하는 힌트 데이터로서 사용할 것..

- 이 인터페이스에 두 가지 유형이 있다.
    1. target steering & velocity/acc
    2. 일반화된 target command interface (페달, 핸들, ..?)
        - 더 동적이고 adaptable한 control scheme를 소개할 수 있어요
        - 1번 유형처럼 원시적 형태보다는, 인간에게 직관적인 형태로 운전 경험을 제공한다
- 또한 이 두 가지를 적재적소에 섞어 쓸 수도 있다.

- Vehicle Interface의 subscribe:
    - /control/command/control_cmd: `autoware_auto_control_msgs/msg/AckermannControlCommand`
    - /control/command/gear_cmd: `autoware_auto_vehicle_msgs/msg/GearCommand`
    - /control/current_gate_mode: `tier4_control_msgs/msg/GateMode`
        - GateMode란? 차량을 제어하는 주체를 명시한다 (autoware, external(외부 조종기 같은거), manual(사람))
    - /control/command/emergency_cmd: `tier4_vehicle_msgs/msg/VehicleEmergencyStamped`
    - /control/command/turn_indicators_cmd: `autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand`
    - 경고등, 액츄에이션, 와이퍼 등등..
- Vehicle Interface의 publish:
    - /vehicle/status/battery_charge: `tier4_vehicle_msgs/msg/BatteryStatus`
    - /vehicle/status/control_mode: `autoware_auto_vehicle_msgs/msg/ControlModeReport`
    - /vehicle/status/gear_status: `autoware_auto_vehicle_msgs/msg/GearReport`
    - /vehicle/status/turn_indicators: `autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport`
    - /vehicle/status/velocity_status: `autoware_auto_vehicle_msgs/msg/VelocityReport`
    - /vehicle/status/hazard_lights_status: `autoware_auto_vehicle_msgs/msg/HazardLightsReport`
    - /vehicle/status/steering_status: `autoware_auto_vehicle_msgs/msg/SteeringReport`
    - ..

이러한 토픽들이 있으니, 구독하고 발행하는 노드를 만드세요.

```cpp
// YOUR-OWN-VEHICLE-INTERFACE.hpp
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
...

class <YOUR-OWN-INTERFACE> : public rclcpp::Node
{
public:
    ...
private:
    ...
    // from autoware
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
    ...
    // from vehicle
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
    ...
    // autoware command messages
    ...
    autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
    ...
    // callbacks
    ...
    void callback_control_cmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
    ...
    void to_vehicle();
    void from_vehicle();
}
```

```cpp
// YOUR-OWN-VEHICLE-INTERFACE.cpp
#include <YOUR-OWN-VEHICLE-INTERFACE>/<YOUR-OWN-VEHICLE-INTERFACE>.hpp>
...

<YOUR-OWN-VEHICLE-INTERFACE>::<YOUR-OWN-VEHICLE-INTERFACE>()
: Node("<YOUR-OWN-VEHICLE-INTERFACE>")
{
  ...
  /* subscribers */
  using std::placeholders::_1;
  // from autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&<YOUR-OWN-VEHICLE-INTERFACE>::callback_control_cmd, this, _1));
  ...
  // to autoware
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  ...
}

void <YOUR-OWN-VEHICLE-INTERFACE>::callback_control_cmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;
}

void <YOUR-OWN-VEHICLE-INTERFACE>::to_vehicle()
{
  ...
  // you should implement this structure according to your own vehicle design
  control_command_to_vehicle(control_cmd_ptr_);
  ...
}

void <YOUR-OWN-VEHICLE-INTERFACE>::to_autoware()
{
  ...
  // you should implement this structure according to your own vehicle design
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
  convert_gear_status_to_autoware_msg(gear_report_msg);
  gear_status_pub_->publish(gear_report_msg);
  ...
}
```

```
<your-autoware-dir>/
└─ src/
    └─ vehicle/
        ├─ external/
+       │   └─ <YOUR-VEHICLE-NAME>_interface/
+       │       ├─ src/
+       │       └─ launch/
+       │            └─ my_vehicle_interface.launch.xml
+       └─ <YOUR-VEHICLE-NAME>_launch/ (COPIED FROM sample_vehicle_launch)
+           ├─ <YOUR-VEHICLE-NAME>_launch/
+           │  ├─ launch/
+           │  │  └─ vehicle_interface.launch.xml
+           │  ├─ CMakeLists.txt
+           │  └─ package.xml
+           ├─ <YOUR-VEHICLE-NAME>_description/
+           │  ├─ config/
+           │  ├─ mesh/
+           │  ├─ urdf/
+           │  │  └─ vehicle.xacro
+           │  ├─ CMakeLists.txt
+           │  └─ package.xml
+           └─ README.md
```

### Ackermann kinematic model

```
AckermannControlCommand
  builtin_interfaces/Time stamp
  autoware_auto_control_msgs/AckermannLateralCommand lateral
    builtin_interfaces/Time stamp
    float32 steering_tire_angle
    float32 steering_tire_rotation_rate
  autoware_auto_control_msgs/LongitudinalCommand longitudinal
    builtin_interfaces/Time stamp
    float32 speed
    float32 accelaration
    float32 jerk
```

### 다른 주행 모델

단순 아커만, 4륜을 넘어서, 다양한 차체의 주행을 지원한다.

차동 조향(differential drive) 차량에 대한 커스터마이징.

- autoware의 아커만 제어 명령을 -> 차동 조향 명령(좌측 바퀴 속도, 우측 바퀴 속도)로 변환하는 vehicle_interface를 만드세요.
- 아커만 조향의 조향 방식 = 앞바퀴 각도 변경
  - 회전 반경 = 고정
  - 제자리 회전 = 불가
- 차동 조향 = 좌우 바퀴 속도 차이로 조향 조절
  - 회전 반경 = 가변
  - 제자리 회전 = 가능

```cpp
// 아커만 → 차동 조향 변환 예시
left_wheel_velocity = longitudinal_speed - (steering_angle * wheel_tread) / 2;
right_wheel_velocity = longitudinal_speed + (steering_angle * wheel_tread) / 2;
```

## 4. creating maps

### intro

- pcd 맵을 만드는 방법?
  - 전통적으로 MMS(mobile mapping system) highly accurate large-scale point cloud maps 만들 수 있었다. 그러나 high-end 센서+정밀한 포지셔닝 필요해서 비쌈
  - 그래서 등장한건 SLAM : 기록된 lidar scan으로 맵 생성!
- 벡터 맵 만드는 방법?
  - bag2lanelet : real이든 sim이든 self-locations data로부터 가상 차선을 생성해준다.
    - 단일 차선만 생성하는 등, 제약이 많다. 다음 섹션에서 소개할 'Vector Map Builder'를 추천한다.
    - https://tools.tier4.jp/vector_map_builder_ll2/
  - MapToolbox 유니티 플러그인 : lanelet2 디자인에 특화
  - JOSM 툴 : 꽤 tedious하고 시간소모되니 사용에 주의할 것
- autoware-compatible map providers
  - MAP IV, Inc. - SLAM - pcd & vector
  - AISAN TECHNOLOGY CO., LTD. - MMS - pcd & vector
  - TomTom - MMS - vector

### SLAM algorithms

### converting UTM maps to MGRS map format

- (우리 프로젝트와 큰 관련 없음)
- MGRS = Military Grid Reference System
- PCL = point cloud library (오픈소스) 사용
  - 이 분야의 독보적인 de facto standard, 사실상 PCL 보통명사화
  - 필터링, 특징 추출, 모델 추정, 분할, 등록, 표면 재구성, 인식, 검색
  - 독보적 1황까지는 아니여도, 교과서 급
- geographiclib 사용

### pointcloud map downsampling

- 300mb를 초과한다던지 하면, 크기를 줄이는 걸 고려해보자
  - 또한 dynamic map loading with partial loading도 고려해볼 만 함.
    - core/map/autoware_map_loader_package 참조하셈
- `sudo snap install cloudcompare` 설치해서 3가지 방법으로 subsampling 가능하다

### creating a vector map (**중요**)

TIER IV's Vector Map Builder 툴을 사용해 Lanelet2 맵을 만드는 걸 설명하겠습니다. (웹 브라우저 기반 앱)

#### lanelet2

```
<YOUR-MAP-DIRECTORY>/
 ├─ pointcloud_map.pcd
 └─ lanelet2_map.osm
```

```xml
<!-- autoware.launch.xml -->
  <arg name="lanelet2_map_file" default="lanelet2_map.osm" description="lanelet2 map file name"/>
  <arg name="pointcloud_map_file" default="pointcloud_map.pcd" description="pointcloud map file name"/>
```

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=<YOUR-MAP-FOLDER-DIR> vehicle_model:=<YOUR-VEHICLE-MODEL> sensor_model:=<YOUR-SENSOR-KIT>
```

## 5. launch autoware

- 다음 하위 모듈들을 런치해줘요:
  - vehicle
  - system
  - map
  - sensing
  - localization
  - perception
  - planning
  - control
- 실제 차량에 대해 런치하기 위해 추가 pre-step 필요:
  - autoware meta-repo 만들기
  - vehicle & sensor model 만들기
  - 센서 calibration 하기
  - autoware-compatible vehicle interface 만들기
  - map 만들기

```xml
<!-- autoware.launch.xml -->
  <arg name="launch_vehicle" default="true" description="launch vehicle"/>
  <arg name="launch_system" default="true" description="launch system"/>
  <arg name="launch_map" default="true" description="launch map"/>
  <arg name="launch_sensing" default="true" description="launch sensing"/>
  <arg name="launch_sensing_driver" default="true" description="launch sensing driver"/>
  <arg name="launch_localization" default="true" description="launch localization"/>
  <arg name="launch_perception" default="true" description="launch perception"/>
  <arg name="launch_planning" default="true" description="launch planning"/>
  <arg name="launch_control" default="true" description="launch control"/>
```

필요 없는 모듈은 제거해도 됩니다. 아니면 cli에서 `launch_control:=false` 이런식으로 인자 전달해도 되고..

```
<YOUR-OWN-AUTOWARE-DIR>/
  └─ src/
       └─ launcher/
            └─ autoware_launch/
                 ├─ config/
                 ├─     ├─ control/
                 ├─     ├─ localization/
                 ├─     ├─ map/
                 ├─     ├─ perception/
                 ├─     ├─ planning/
                 ├─     ├─ simulator/
                 ├─     └─ system/
                 ├─launch/
                 └─ rviz/
```

- 각 모듈 관련 파라미터 파일들은 config/ 디렉토리 아래에 넣으세요
- autoware_launch에서 작성된 파라미터 값들이, 결국 호출할 노드와 패키지에 있는 파라미터들을 모조리 overwrite할 수 있으므로 크게 걱정 안 해도 된다..

- file: autoware.launch.xml (package: autoware_launch)
  - file: vehicle.launch.xml (package: tier4_vehicle.launch.xml)
    - node: robot_state_publisher (package: tier4_vehicle.launch.xml)
    - @if launch_vehicle_interface
      - node: vehicle_interface.launch.xml (package: <YOUR-VEHICLE-LAUNCH>)
  - file: tier4_system_component.launch.xml (package: autoware_launch)
    - file: system.launch.xml (package: tier4_system_launch)
      - @if launch_system_monitor ..
      - @if launch_dummy_diag_publisher ..
      - file: service_log_checker.launch.xml
      - file: component_state_monitor.launch.xml
      - file: system_error_monitor.launch.xml
      - file: emergency_handler.launch.xml
      - file: duplicated_node_checker.launch.xml
      - file: mrm_comfortable_stop_operator.launch.xml
      - file: mrm_emergency_stop_operator.launch.xml
  - file: tier4_map_component.launch.xml (package: autoware_launch)
    - file: map.launch.xml (package: tier4_map_launch)
      - ComposableNode: lanelet2_map_loader (package: map_loader)
      - ComposableNode: lanelet2_map_visualization (package: map_loader)
      - ComposableNode: pointcloud_map_loader (package: map_loader)
      - ComposableNode: vector_map_tf_generator (package: map_tf_generator)
      - Node: map_hash_generator (package: map_loader)
  - file: tier4_sensing_component.launch.xml (package: autoware_launch)
    - file: sensing.launch.xml (package: tier4_sensing_launch)
      - file: sensing.launch.xml (package: <YOUR-SENSOR-KIT>_launch)
        - file: imu.launch.xml
        - file: lidar.launch.xml
        - file: camera.launch.xml
        - file: gnss.launch.xml
  - file: tier4_localization_component.launch.xml (package: autoware_launch)
    - file: localization.launch.xml (package: tier4_localization_launch)
      - file: pose_twist_fuction_filter.launch.xml
        - file: ekf_localizer.launch.xml
        - file: stop_filter.launch.xml
        - file: twist2accel.launch.xml
      - file: pose_twist_estimator.launch.xml
        - @match pose_source
          - ndt
          - yabloc
          - artag
          - eagleye
        - @match twist_source
          - eagleye
          - gyro_odom
      - file: localization_error_monitor.launch.xml
        - file: localization_error_monitor.launch.xml (package: localization_error_monitor)
  - file: tier4_perception_component.launch.xml (package: autoware_launch)
    - file: perception.launch.xml (package: tier4_perception_launch)
      - file: ground_segmentation.launch.xml
      - file: probablistic_occupancy_grid_map.launch.xml
      - @if use_empty_dynamic_object_publisher
        - node: dummy_perception_publisher
      - @else
        - file: detection.launch.xml
          - @match perception_mode
            - Camera-lidar-radar fusion based
            - Camera-lidar fusion based
            - Lidar based
            - Lidar-radar based
            - Radar based
        - file: tracking.launch.xml
        - file: prediction.launch.xml
      - @if use_traffic_light_recognition
        - file: traffic_light.launch.xml
  - file: tier4_planning_component.launch.xml (package: autoware_launch)
    - file: planning.launch.xml (package: tier4_planning_launch)
      - file: mission_planning.launch.xml
      - file: scenario_planning.launch.xml
        - file: scenario_selector.launch.xml
        - file: external_velocity_limit_selector.launch.xml
        - file: lane_driving.launch.xml
          - file: behavior_planning.launch.xml
          - file: motion_planning.launch.xml
        - file: parking.launch.xml
      - file: planning_validator.launch.xml
      - file: planning_evaluator.launch.xml
  - file: tier4_control_component.launch.xml (package: autoware_launch)
    - file: control.launch.py (package: tier4_control_launch)
      - file: external_cmd_selector.launch.pyl
      - Container: control_container
        - ComposableNode: controller_node_exe
        - ComposableNode: lane_departure_checker_node
        - ComposableNode: control_validator
        - ComposableNode: shift_decider
        - ComposableNode: glog_component
        - ComposableNode: operation_mode_transition_manager
        - @if enable_autonomous_emergency_braking
          - ComposableNode: autonomous_emergency_braking
        - @if enable_predicted_path_checker
          - ComposableNode: predicted_path_checker
        - @if enable_obstacle_collision_checker
          - ComposableNode: obstacle_collision_checker
        - ComposableNode: vehicle_cmd_gate
      - file: external_cmd_converter.launch.py

```xml
<arg name="launch_vehicle_interface" default="false" description="launch vehicle interface"/>
<arg name="system_error_monitor_param_path" default="<YOUR-SYSTEM-ERROR-PARAM-PATH>"/>
<arg name="lanelet2_map_file" default="<YOUR-LANELET2-MAP-NAME>" description="lanelet2 map file name"/>
<arg name="pointcloud_map_file" default="<YOUR-PCD-FILE-NAME>" description="pointcloud map file name"/>
<arg name="launch_sensing_driver" default="false" description="launch sensing driver"/>
<arg name="pose_source" default="eagleye" description="select pose_estimator: ndt, yabloc, eagleye"/>
<arg name="twist_source" default="eagleye" description="select twist_estimator. gyro_odom, eagleye"/>
<arg name="input_pointcloud" default="/sensing/lidar/concatenated/pointcloud"/>
<arg name="input_vehicle_twist_with_covariance_topic" value="<YOUR-VEHICLE-TWIST-TOPIC-NAME>"/>
<arg name="occupancy_grid_map_method" default="laserscan_based_occupancy_grid_map" description="options: pointcloud_based_occupancy_grid_map, laserscan_based_occupancy_grid_map"/>
<arg name="detected_objects_filter_method" default="position_filter" description="options: lanelet_filter, position_filter"/>
<arg name="occupancy_grid_map_method" default="laserscan_based_occupancy_grid_map" description="options: pointcloud_based_occupancy_grid_map, laserscan_based_occupancy_grid_map"/>
<arg
  name="detected_objects_validation_method"
  default="occupancy_grid"
  description="options: obstacle_pointcloud, occupancy_grid (occupancy_grid_map_method must be laserscan_based_occupancy_grid_map)"
  />
<arg name="use_experimental_lane_change_function" default="false"/>
<arg name="cruise_planner_type" default="obstacle_cruise_planner" description="options: obstacle_stop_planner, obstacle_cruise_planner, none"/>
<arg name="use_surround_obstacle_check" default="false"/>
<arg name="velocity_smoother_type" default="L2" description="options: JerkFiltered, L2, Analytical, Linf(Unstable)"/>
<arg name="lateral_controller_mode" default="pure_pursuit"/> <!-- 또는 mpc -->
<arg name="enable_autonomous_emergency_braking" default="true"/>
<arg name="enable_predicted_path_checker" default="true"/>
```

- eagleye에 대하여! lidar+pcd가 아닌, gnss/imu 기반의 MAP IV. Inc.가 개발한 측위 기술! cost-effective, 저비용
  - https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/launch-autoware/localization/eagleye/

## 6. tuning parameters and performance

## 7. AWSIM integration

## 8. integrating sensors

## Appendix A. Machine learning models

## Appendix B. Others


# autoware universe documentation