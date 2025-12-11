# ROS2 Autonomous Navigation Stack 프로토타입 명세서 템플릿

## 📋 프로토타입 개요

### 기본 정보
- **프로토타입 이름**: [프로토타입 식별명]
- **생성일**: [생성일자]
- **버전**: v1.0
- **목표**: [간단한 목표 설명]

### 아키텍처 철학
```
[해당 프로토타입의 설계 철학과 접근 방식 설명]
```

## 🏗 시스템 아키텍처

### 전체 데이터 흐름도
```
Map Server → Global Planner → Local Planner → Controller → Actuator
        ↓              ↓              ↓
     Localization ← Sensor Fusion ← Perception
```

## 📊 단계별 상세 명세

### 1. 맵 처리 (Map Processing)
**담당 노드**: `map_server_node`

#### 알고리즘
- **주요 알고리즘**: [예: Occupancy Grid, Costmap]
- **대체 알고리즘**: [예: OctoMap, Voxel Grid]
- **선택 이유**: [해당 알고리즘 선택 이유]

#### 인터페이스
```yaml
# 발행 토픽
- /map: nav_msgs/msg/OccupancyGrid
  - 맵 데이터 (해상도: 0.05m, 크기: 400x400)
- /map_metadata: nav_msgs/msg/MapMetaData

# 구독 토픽
- /initialpose: geometry_msgs/msg/PoseWithCovarianceStamped
```

#### 파라미터
```yaml
map_server:
  ros__parameters:
    map_file: "maps/prototype_map.yaml"
    frame_id: "map"
    resolution: 0.05
```

### 2. 글로벌 경로 계획 (Global Planning)
**담당 노드**: `global_planner_node`

#### 알고리즘
- **주요 알고리즘**: A*
- **대체 알고리즘**: Dijkstra, RRT*, Hybrid A*
- **선택 이유**: 최적성 보장과 효율성의 균형

#### 인터페이스
```yaml
# 발행 토픽
- /global_plan: nav_msgs/msg/Path
  - 글로벌 경로 waypoints
- /global_costmap: nav2_msgs/msg/Costmap

# 구독 토픽
- /map: nav_msgs/msg/OccupancyGrid
- /goal_pose: geometry_msgs/msg/PoseStamped
- /amcl_pose: geometry_msgs/msg/PoseWithCovarianceStamped
```

#### 파라미터
```yaml
global_planner:
  ros__parameters:
    planner_type: "a_star"
    heuristic_type: "euclidean"
    interpolation_resolution: 0.1
    default_tolerance: 0.5
```

### 3. 지역 경로 계획 (Local Planning)
**담당 노드**: `local_planner_node`

#### 알고리즘
- **주요 알고리즘**: [예: DWA, TEB]
- **대체 알고리즘**: [예: MPC, EBand]
- **선택 이유**: [선택 이유]

#### 인터페이스
```yaml
# 발행 토픽
- /local_plan: nav_msgs/msg/Path
- /local_costmap: nav2_msgs/msg/Costmap
- /cmd_vel_unfiltered: geometry_msgs/msg/Twist

# 구독 토픽
- /global_plan: nav_msgs/msg/Path
- /scan: sensor_msgs/msg/LaserScan
- /odom: nav_msgs/msg/Odometry
- /amcl_pose: geometry_msgs/msg/PoseWithCovarianceStamped
```

#### 파라미터
```yaml
local_planner:
  ros__parameters:
    planner_frequency: 10.0
    max_vel_x: 0.5
    max_vel_theta: 1.0
    acc_lim_x: 0.5
    acc_lim_theta: 1.0
    inflation_radius: 0.3
```

### 4. 제어기 (Controller)
**담당 노드**: `controller_node`

#### 알고리즘
- **주요 알고리즘**: MPC + Pure Pursuit + PID (하이브리드)
- **대체 알고리즘**: Stanley, LQR, Pure PID
- **선택 이유**: 장기 최적화 + 경로 추종 + 안정성 보장

#### 인터페이스
```yaml
# 발행 토픽
- /cmd_vel: geometry_msgs/msg/Twist
- /control_debug: custom_msgs/msg/ControlDebug

# 구독 토픽
- /local_plan: nav_msgs/msg/Path
- /odom: nav_msgs/msg/Odometry
- /vehicle_status: custom_msgs/msg/VehicleStatus
```

#### 파라미터
```yaml
controller:
  ros__parameters:
    # MPC 파라미터
    mpc_horizon: 10
    mpc_dt: 0.1
    mpc_max_steer: 0.5
    
    # Pure Pursuit 파라미터
    lookahead_distance: 1.0
    min_lookahead: 0.5
    max_lookahead: 2.0
    
    # PID 파라미터
    kp_linear: 0.8
    ki_linear: 0.01
    kd_linear: 0.1
    kp_angular: 1.2
    ki_angular: 0.02
    kd_angular: 0.15
```

## 🔄 추가 단계 (필요시)

### 5. 센서 퓨전 (Sensor Fusion)
**담당 노드**: `sensor_fusion_node`

#### 알고리즘
- **주요 알고리즘**: Extended Kalman Filter
- **대체 알고리즘**: Particle Filter, Unscented Kalman Filter

#### 인터페이스
```yaml
# 발행 토픽
- /fused_odometry: nav_msgs/msg/Odometry

# 구독 토픽
- /odom/wheel: nav_msgs/msg/Odometry
- /imu/data: sensor_msgs/msg/Imu
- /gps/fix: sensor_msgs/msg/NavSatFix
```

### 6. 위치 추정 (Localization)
**담당 노드**: `localization_node`

#### 알고리즘
- **주요 알고리즘**: AMCL
- **대체 알고리즘**: EKF, Particle Filter

## 📈 성능 메트릭스

### 평가 지표
```yaml
navigation_metrics:
  - success_rate: 95%
  - average_speed: 0.8 m/s
  - path_deviation: < 0.1m
  - computation_time: < 50ms
  - recovery_behavior: enabled
```

## 🚀 실행 방법

### 단일 명령어 실행
```bash
ros2 launch prototype_[name] full_stack.launch.py
```

### 단계별 실행
```bash
# 1. 맵 서버
ros2 run map_server map_server --ros-args -p yaml_filename:="maps/prototype.yaml"

# 2. 글로벌 플래너
ros2 run global_planner global_planner_node

# 3. 지역 플래너 + 라이다
ros2 run local_planner local_planner_node
ros2 run rplidar_ros rplidar_node

# 4. 제어기
ros2 run controller controller_node

# 5. 로봇 시작
ros2 run robot_driver robot_base_node
```

## 🔧 튜닝 가이드

### 주요 튜닝 파라미터
```yaml
critical_parameters:
  global_planner:
    - cost_scaling_factor
    - neutral_cost
  local_planner:
    - max_vel_x
    - acc_lim_x
    - inflation_radius
  controller:
    - lookahead_distance
    - mpc_horizon
    - pid_gains
```

### 디버깅 툴
```bash
# RViz2 시각화
ros2 run rviz2 rviz2 -d src/prototype/config/navigation.rviz

# 토픽 모니터링
ros2 topic echo /cmd_vel
ros2 topic hz /scan

# 파라미터 튜닝
ros2 param set /local_planner max_vel_x 0.8
```

---

























## 📝 예제: MPC + Pure Pursuit + PID 하이브리드 제어기

### 제어기 아키텍처 상세
```
입력: /local_plan (경로), /odom (현재 상태)
↓
MPC: 장기 궤적 최적화 (10스텝 예측)
↓
Pure Pursuit: 경로 추종 (Lookahead Point 기반)
↓  
PID: 속도/조향각 안정화
↓
출력: /cmd_vel (제어 명령)
```

### 토픈 상세 명세
```yaml
# common_interfaces 기반 상세 토픽
cmd_vel:
  type: geometry_msgs/msg/Twist
  fields:
    linear:
      x: float64  # 전진 속도 (m/s)
      y: float64  # 측면 속도 (0)
      z: float64  # 수직 속도 (0)
    angular:
      x: float64  # 롤 각속도 (0)
      y: float64  # 피치 각속도 (0)
      z: float64  # 요 각속도 (rad/s)

odom:
  type: nav_msgs/msg/Odometry
  fields:
    pose: 
      position: {x: float64, y: float64, z: float64}
      orientation: {x: float64, y: float64, z: float64, w: float64}
    twist:
      linear: {x: float64, y: float64, z: float64}
      angular: {x: float64, y: float64, z: float64}

scan:
  type: sensor_msgs/msg/LaserScan
  fields:
    angle_min: float32
    angle_max: float32
    angle_increment: float32
    range_min: float32
    range_max: float32
    ranges: float32[]
    intensities: float32[]
```

이 템플릿을 기반으로 각 프로토타입별로 세부 내용을 채워나가시면 됩니다. 특히 알고리즘 선택 이유와 파라미터 튜닝 경험을 상세히 기록하는 것이 중요합니다.