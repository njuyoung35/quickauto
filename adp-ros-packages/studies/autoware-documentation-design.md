# autoware design

## (overall) design

### arch

- sensing :camera + lidar + radar + gnss
- computing : 측위, 인지, 계획, control, 로깅, 시스템 모니터링
- actuaction : DBW component
- tools : 시뮬레이터, mapping, 원격조종, ML, annotation, calibration

### design

- autoware components
- autoware architecture
- autoware interface

## concepts (components)

- 오픈소스
- 포괄적인 기능 제공
    - ADS을 위한 키 알고리즘
    - HW 통합
    - 시뮬레이션 지원
    - 도구 - sensor calibration, mapping, data creation, diagnostic, and scenario tests.
- microautonomy arch
    - 모듈화되어 있어서 갈아낄 수 있다.
    - 내부적으로는 component interface 따라서 통신
    - 외부적으로는 AD API 노출을 통해 외부 앱과 호환
- core & universe package management
    - core : AWF에서 퀄리티를 보장하는 티어
    - universe : 오픈소스 개발, 열린 공간의 티어
    - 이렇게 two-tier 시스템으로 나눠 관리한다.

## arch

### sensing

### mapping

- 요구사항
    - HD 의미론 벡터맵
    - PCD 같은 지형적 정보 (옵셔널)
- arch
    - input : pcd map, vector map, projection info
        - projection info : local coordinates(x, y, z)와 geodetic coordinate(lat, lon, alt) 사이 변환 정보
    - output : 측위, 인지 컴포넌트에게..
- map component interface
    - 맵 컴포넌트에 대하여..
    - input : from file system
    - output : 센싱(proj info), 측위, 인지, 계획(벡터맵), API layer(proj info)
- map specification
    - pcd map
    - vector map
        - lanelet2
    - proj info
        - .yaml, `map_projection_loader`에게 제공되어야 한다.

#### vector map requirements overview

https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/map-requirements/vector-map-requirements-overview/

```
Category Lane 	vm-01-01 	Lanelet basics
	vm-01-02 	Allowance for lane changes
	vm-01-03 	Linestring sharing
	vm-01-04 	Sharing of the centerline of lanes for opposing traffic
	vm-01-05 	Lane geometry
	vm-01-06 	Line position (1)
	vm-01-07 	Line position (2)
	vm-01-08 	Line position (3)
	vm-01-09 	Speed limits
	vm-01-10 	Centerline
	vm-01-11 	Centerline connection (1)
	vm-01-12 	Centerline connection (2)
	vm-01-13 	Roads with no centerline (1)
	vm-01-14 	Roads with no centerline (2)
	vm-01-15 	Road shoulder
	vm-01-16 	Road shoulder Linestring sharing
	vm-01-17 	Side strip
	vm-01-18 	Side strip Linestring sharing
	vm-01-19 	Walkway
Category Stop Line 	vm-02-01 	Stop line alignment
	vm-02-02 	Stop sign
Category Intersection 	vm-03-01 	Intersection criteria
	vm-03-02 	Lanelet's turn direction and virtual
	vm-03-03 	Lanelet width in the intersection
	vm-03-04 	Lanelet creation in the intersection
	vm-03-05 	Lanelet division in the intersection
	vm-03-06 	Guide lines in the intersection
	vm-03-07 	Multiple lanelets in the intersection
	vm-03-08 	Intersection Area range
	vm-03-09 	Range of Lanelet in the intersection
	vm-03-10 	Right of way (with signal)
	vm-03-11 	Right of way (without signal)
	vm-03-12 	Right of way supplements
	vm-03-13 	Merging from private area, sidewalk
	vm-03-14 	Road marking
	vm-03-15 	Exclusive bicycle lane
Category Traffic Light 	vm-04-01 	Traffic light basics
	vm-04-02 	Traffic light position and size
	vm-04-03 	Traffic light lamps
Category Crosswalk 	vm-05-01 	Crosswalks across the road
	vm-05-02 	Crosswalks with pedestrian signals
	vm-05-03 	Deceleration for safety at crosswalks
	vm-05-04 	Fences
Category Area 	vm-06-01 	Buffer Zone
	vm-06-02 	No parking signs
	vm-06-03 	No stopping signs
	vm-06-04 	No stopping sections
	vm-06-05 	Detection area
Category Others 	vm-07-01 	Vector Map creation range
	vm-07-02 	Range of detecting pedestrians who enter the road
	vm-07-03 	Guardrails, guard pipes, fences
	vm-07-04 	Ellipsoidal height
```

### localization

### perception

- 필요한거
    - 정적 데이터 - pcd map, vector map
    - 초실시간성 - vehicle odom
    - 센서 실시간 데이터 - pcd, lidar, radar, gnss ..
- 내보내는거
    - occupancy grid
    - dynamic objects (ml 돌려서 액션 추적까지)
    - obstacle points
    - traffic lights
    - 이거를 planning한테 보내면 된다.
- subcomponents
    - obstacle segmentation : 정적, 동적이든 실시간 pcd에 보이는거 식별하자.
    - occupancy grid map : 사각지대를 감지하자.
        - ego 차량의 한계를 인정하고, dynamic objects가 튀어나올 가능성을 고려하자
        - 점유격자라기 보다도.. '나의 인지 범위'를 추상화한 자료구조일 듯
    - object regonition : 동적 물체를 인식해서, 미래 궤적을 예측하자
        - detection : 차량과 보행자의 odom을 감지하자
            - detector : 프레임마다 obj detection 시도
            - interpolator : detector의 실시간 보고가 끊기더라도, 보정을 통해 양상 추적을 이어가자
        - tracking : 여러 프레임동안 누적된 detect 결과와 관련
        - prediction
    - traffic light recognition : 신호등 색과, 화살표 시그널의 방향을 인식하자
- base 3d detection : aw는 기본적으로 ml 기반
    - CenterPoint
    - TransFusion-L
    - BEVFusion-L
    - apollo instance segmentation + shape estimation
        - 기본적으로 3d 감지 범위는 90~120m 사이
- near-object 3d detection
    - 전자는 큰 범위에서의 감지였고 측위나, 거시적 양상 파악에 좋았다면, 근접에서의 비상 상황 감지에 특화된 단위를 또한 고려해야 한다.
    - 고새항도 voxel grid + ML, 작은 객체에 대한 탐지 정확도도 올리자
    - 주로 30~50m 탐지 거리
- (TBD) camera-only 3d detection
- radar-only faraway object 3d detection
    - radar는 전파를 사용해서 더 먼, 더 거시적 측면에서 주변을 감지할 수 있다.
    - 추가적으로 날씨에 강건함
- (TBD) 3d semantic segmentation
- cluster-based 3d detection
    - roi based pc fusion을 포함한 유클리디안 클러스터링 기반
    - 계산량 많아서, 근접 급박 상황에 대해서 이 파이프라인 사용한는 건 지양
- multi-object tracking v2
    - priority object merger
        - 데이터를 매번 다 보내기에는 한계가 있다. 우선순위를 부여해 중요 감지 데이터부터 보내도록 하자
    - stationary detection
        - 정적 장애물을 인식하고 나서는, 고빈도 계산을 모든 것에 적용하지 말고 선택적으로 적용하자

### planning

개요 : 요기 문서는 두 파트로 분할됨
    - 고수준 요구사항과 디자인 논의
    - 실제 구현과 구체 기능

#### goals and non-goals

우리의 목표는 단순히 하나의 ADS를 구현하는 데 멈추지 않고, autonomous riving platform을 구축하는 데 있다. 그리하여 개별 vehicle의 니즈에 따라 기능을 쌓아 올릴 수 있도록.

그리하여 autoware는 구체적 시나리오에 대응되는 정책을 제공하기보다, customizable and easily estendable planning dev platform을 제공한다.

goals:
    - 기본기능 제공되어서, 간단한 ODD를 정의할 수 있어요
        - moving, stopping, turning 등 기본 기능 제공
        - as well as handling lane changes, and obstacle avoidance까지 고려한..
    - 기능은 모듈성, 확장성 좋음
        - 다양한 ODD에 적용될 수 있음. 정말 광범위 의미의 'vehicle'에 대해!
        - 구체 ODD에 대한 코드는 제공 되지 않음
    - 인간 오퍼레이터의 의사결정에 따라 능력을 확장시킬 수 있음
nongoals:
    - 모든 유저-요구사항 만족
    - 완전한 기능성과 성능을 제공하기 (애초에 플랫폼임!)
    - 절대적 안저 보장

#### high level design

- 인풋 받아오는 곳:
    - map, 인지, 측위
- 상호작용 하는 곳:
    - api layer, human machine interface, system component
- 출력 내뱉는 곳:
    - control component

- planning component 구성 (위에서 아래로 흐름)
    - mission planning
        - route planning
        - goal planning
        - 맵 데이터를 활용하자. FMS (fleet management system), car nagivation route planning과 유사
    - behavior & motion
        - behavior : safe & rule-compliant routes 계산에 집중
            - lane change
            - intersection
            - crosswalk
            - stop line
            - avoidance
            - pull over
            - parking
        - motion : 물리적 모션, 궤적, 승차감에 대한 계산
            - path planning
            - velocity planning
            - drivable area
            - freespace
            - obstacle stop
            - slow down
            - ML planner
    - valdiation : 계획된 궤적에 대해 평가. 비상 반응 포함 (이전 파이프라인으로 돌아가서 대안 궤적 생성 요구)
        - collision check
        - TTC check (신호등?)ㄴ
        - comfortability
    - 끝, 컨트롤에게 전달!
- autoware의 기가막힌 모듈성으로 인해, 이거를 진짜 하나부터 열까지 다 갈아끼울 수 있는 단위로 운용할 수 있다.
    - 전체 planning component 바꿔끼기
    - subcomponent 바꿔끼기
    - module 바꿔끼기

#### io 인터페이스 상세

- input
    - map - 벡터맵 : 정적 정보
    - perception : 이전 인지 챕터에서 알아본 네 가지 정보
        - obj info, obstacle info, 점유격자, 신호등
    - 측위 - odom, veloc, acc 등 motion-related data
    - 시스템 - operation mode
    - HMI - 차선변경, 교차로 진입 등 고수준 오퍼레이션 지시
    - api layer
        - 목적지 설정
        - checkpoint 전달. 목적지로 이어지는 중단단계
        - velocity limit
- output
    - control
        - 궤적 : 부드러운 pose 및 가속도의 시퀀스. 0.1초 해상도로 10초 범위 제공 (mpc가 써먹으라고..)
        - turn signal : 차량 깜빡이 켜야하니까..
    - system
        - 진단메시지 : planning 검증 단계에서 얻은 정보 같은거..
    - HMI - 상태 보고
        - 궤적 후보 선택지 제시
    - api layer
        - planning factors : 내가 이 결정을 내린 것에 대한 근거를 제시. 탐지된 객체, 장애물들 디버깅 같은 느낌?

- 내부 메시지 전달은?
    - mission planning - > scenario planning
        - route : 출발지에서 목적지로 향하기 위한 가이던스. lane IDs 같은 정보로 정의됨. 구체 차선을 제시하진 않음. 차도 제시
    - behavior planning -> motion planning
        - path : 지점에 대해 구체적 위치와 속도 상태를 할당. 주로 1 meter 간격으로 제공됨.
        - driving area : 법적 제한구역, 물리적 운전가능 구역 등을 정의. 모션 플래너가 이걸 고려할 것임
    - scenario planning -> valdiation
        - trajectory : 0.1초 간격, 요구되는 pos, vel, acc를 정하자.
    - valdiation -> control component
        - trajectory : safety 고려사항 추가해서 ㄱㄱ

#### supported features

- route planning
    - 에고 차량 위치에서, 목적지까지의 루트 설계
    - `mission_planner`에서 수행
    - requirements: lanelet map (driving lanelets)
- path palnning from route
    - 주어진 루트를 따르는 경로 설계
    - Behavior Path Planner에서 수행
    - requirements: lanelet map (driving lanelets)
- obstacle avoidance
    - 조향 기능으로 장애물을 피하는 경로 설계
    - Static Avoidance Module + Path Optimizer에서 수행
    - requirements: objects information
- path smoothing
    - 부드러운 조향 달성을 위한 경로 설계
    - Path Optimizer에서 수행
    - requirements: lanelet map (driving lanelets)
- narrow space driving
    - drivable area에서 주행하도록 경로 설계
        - 만약 영역을 벗어날 수 밖에 없다면, 그렇지 말도록 정지 계획을 세우자.
    - Path Optimizer에서 수행
    - requirements: Lanelet map (high-precision lane boundaries)
    - parking도 그렇고.. 좁은 구역에서는 후진도 고려하도록 설계해야하나...?
- lane change
    - requirements: Lanelet map (driving lanelets)
    - Lane Change에서 수행
- pull over
    - 차를 세우기 - simple / arc forward / arc backward
    - Goal Planner에서 수행
    - requirements: Lanelet map (shoulder lane)
- pull out
    - 전자와 반대로, 주정차한 상태에서 출발하는 것
    - Start Planner에서 수행
    - requirements: Lanelet map (shoulder lane)
- path shift
    - 원하는 lateral offset input을 수행한다.
    - 추가 상위 명령 이전에 저수준에서의 lateral shift를 수행하는 모듈
    - Side Shift Module에서 수행
- obstacle stop
    - 경로 상 장애물에 대응하여 속도를 감속시시켜 정지하기
    - Obstacle Stop Planner, Obstacle Cruise PLanner에서 수행
    - requirements: objects information
- obstacle deceleration
    - 경로 좌우로 장애물이 있을 경우, 혹시 모를 충돌을 대비해 속도를 감속시키기
    - Obstacle Stop Planner, Obstacle Cruise PLanner에서 수행
    - requirements: objects information
- Adaptive Cruise Control
    - 에고 차량 전방 주행 차량의 속도를 따라 주행하도록 계획 짜기
    - Obstacle Stop Planner, Obstacle Cruise Planner에서 수행
    - requirements: objects information
- Decelerate for cut-in vehicles
    - 에고 차선으로 끼어들기 하는 차량에 대응하여 감속하기
    - Obstacle Cruise Planner에서 수행
    - requirements: objects information
- Surround check at starting
    - 

#### Reference Implementation

node diagram.. 이런식으로 만드시면 돼요~의 autoware가 제공하는 example

#### important information in the current implementation

#### customize features in the current implementation

#### 

### control

### vehicle

### node diagram..!


## interface

### AD API

### components

## configuration management

# tier4_state_rviz_plugin/AutowareStatePanel

- OperationMode
- AutowareControl
- Routing
- Localization
- Motion
- FailSafe
- Gear
- Send Velocity Limit
- Set Emergency