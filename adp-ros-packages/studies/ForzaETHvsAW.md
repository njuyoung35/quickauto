ForzaETH Spliner / Global Planner(mincurv/mintime) / L1 Controller / Particle Filter 구조  >>  Autoware의 다음 구조


## 1. Mission Planning

Autoware: Route Planning, Goal Planning 등 “어디로 갈지” 정하는 상위 네비 레벨.

ForzaETH에서는 거의 고정값으로 취급되고, 아래처럼 대응된다.

Global map + raceline + 섹터 파일들

maps/teras_*.pgm, global_waypoints.json,
speed_scaling.yaml, ot_sectors.yaml, ctrl_sectors.yaml 등

“이 트랙을 이 레이스라인으로 한 바퀴 돈다”가 이미 정해져 있어서
Autoware의 Mission Planning처럼 매번 라우팅을 하지 않고, 사전에 만들어 둔 route를 그대로 사용.

실제 “목적지 선택” UI나 라우팅 알고리즘은 없음

그래서 Mission Planning 레이어는 정적 설정(offline tool) 이라고 보면 된다.

즉, ForzaETH는 “Mission Planning이 필요 없는 고정 서킷 자율주행” 상황이라
미리 만든 레이스라인/섹터들이 Mission Planning 결과를 대신한다고 보면 돼.


## 2. Behavior 레이어 대응

다이어그램: Lane Change, Intersection, Crosswalk, Stop Line, Avoidance, Pull Over, Parking …

ForzaETH에서 Behavior에 해당하는 건:

### 2.1 State Machine (핵심)

위치: stack_master/config/state_machine_params.yaml + 관련 코드

상태 예시: GBTRACK, TRAILING, OVERTAKE, SPLINE_LEFT/RIGHT 등

하는 일:

“지금 기본 레이스라인을 탈지 / 추월 모드로 갈지 / 회피 스플라인을 쓸지”

H2H에서 상대차 거리·상태에 따라 모드 전환

Autoware 관점:

Lane Change / Avoidance / Obstacle Stop / Slow Down 같은 걸 상태로 관리하는 Behavior 모듈에 해당.

### 2.2 Opponent Estimation (Detect + Tracking)

패키지: Detect.cpp, Tracking.py, opponent_tracking_param_cpp.yaml 등

역할:

라이다 클러스터링으로 차량/장애물 검출

EKF 기반 상대 차량 위치·속도 추정

Behavior와의 관계:

“앞에 차가 얼마나 가까운지, 추월 가능한지” 정보를 State Machine에 공급 →
Autoware의 Perception → Behavior 경로와 동일한 역할.

### 2.3 TinyLidarNet 구간 선택 (PP vs TLN 스위칭)

ctrl_sectors.yaml 로 섹터별 컨트롤러(pp / tln) 선택

특정 구간에서 End-to-End(ML)로 갈지 / 전통 PP로 갈지를 결정

Autoware 다이어그램에서는 Behavior 블록의 “etc…” 또는 특수 Behavior + ML Planner 선택 로직으로 볼 수 있음.

정리하면, State Machine + Opponent Estimation + 컨트롤러 스위치 로직이
Autoware의 Behavior 박스를 구성한다고 보면 된다.


## 3. Motion 레이어 대응

다이어그램: Path Planning, Velocity Planning, Drivable Area, Freespace, Obstacle Stop, Slow Down, ML Planner …

ForzaETH에서는 크게 세 덩어리로 나뉜다.

### 3.1 Global Planner (mincurv / mintime)

패키지: planner/global_planner + racecar_f110.ini, global_planner_params.yaml

기능:

맵에서 최적 raceline 생성 (minimum curvature / minimum time)

track bounds, safety_width, width_opt, penalty_* 등을 이용해 “드라이버블 에어리어 내 최적 경로” 계산

Autoware 대응:

Motion / Path Planning + 부분적으로 Drivable Area 고려에 해당

Mission 보다는 한 단계 아래, “트랙 내부에서 최적 line 찾기”이므로 Motion 쪽으로 보는 게 자연스러움.

### 3.2 Spliner (Local Planner)

패키지: planner/local_planners/spline_planner & spliner README

역할:

전방 장애물(상대차 포함) 주변으로 스플라인 곡선을 생성하여 회피 경로 만들기

evasion_dist, spline_bound_mindist, pre/post_apex_*, kd_obs_pred, fixed_pred_time 등 파라미터로
회피 궤적의 모양과 유효성 결정

Autoware 대응:

Behavior 영역에서 보면 Avoidance / Lane Change에 해당하고,

Motion 영역에서 보면 Local Path Planning 역할.

사실상 다이어그램에서 Behavior–Motion 중간에 있는 “Avoidance + Path Planning” 박스라고 생각하면 된다.

### 3.3 속도 계획(“Velocity Planning”에 해당)

ForzaETH에는 전용 Velocity Planner 노드는 없지만, 다음 요소들이 합쳐져서 그 역할을 한다:

Sector 기반 speed scaling

speed_scaling.yaml : 섹터별 speed_scale 조정

Global waypoint의 기본 속도 위에 섹터별 계수를 곱해 구간별 목표 속도 결정.

L1 Controller 내부 속도 로직

l1_params.yaml 의 speed_lookahead, lat_err_coeff, downscale_factor, speed_lookahead_for_steer 등으로
곡률/횡오차에 따른 속도 감쇠 로직 구현

trailing 모드에서 trailing_gap, trailing_*_gain으로 앞차와 간격 유지 → Autoware의 Obstacle Stop / Slow Down에 해당.

State machine 제한

특정 state(OVERTAKE, TRAILING 등)에 따라 속도 상한/동작이 바뀜.

Autoware의 Velocity Planning + Obstacle Stop + Slow Down이
ForzaETH에서는 섹터 속도 스케일 + L1 내부 로직 + state machine으로 “분산 구현”되어 있는 셈.

### 3.4 ML Planner

ForzaETH 측 대응: TinyLidarNet

2D LiDAR → end-to-end steering & speed 출력

다이어그램의 Motion 레이어 안 ML Planner 박스와 1:1 대응.

단, TLN은 전통 경로 계획을 대체하는 “다른 모드”라서, 섹터별로 PP ↔ TLN 전환.


## 4. Validation 레이어 대응

다이어그램: Collision Check, TTC Check, Comfortability, etc…

ForzaETH에는 “Validation 전용 노드”는 없지만, 여기저기 흩어져 있다:

Spliner 내부 유효성 검사

spline_bound_mindist 조건, track boundary 침범 여부 체크

장애물과의 최소 거리 조건 등으로 “회피 경로 취소” 여부 판단

State Machine + Opponent Estimation 조건

상대차 거리·위치를 보고 추월/추종/중단 여부 결정 → 일종의 simple collision / TTC 체크.

Global Planner 경계 체크

d_left, d_right, safety_width 등을 통해 차폭 + 마진을 보장하게 설정 →
raceline이 벽과 너무 가까워지지 않도록 하는 오프라인 Validation

Autoware만큼 “모든 궤적을 마지막에 한 번 더 검사하는 Validation 블록”은 없고,
각 Planner 내부에서 로컬하게 Validation을 수행한다고 보면 돼.


## 5. Control Component 대응

Autoware: Trajectory를 받아 실제 조향/가속/제동을 출력하는 레이어.

ForzaETH에서 완전히 여기에 해당하는 건:

### 5.1 L1 / MAP / (K)MPC / STMPC Controllers

위치: controller/pp (L1), 기타 controller 패키지들 + l1_params.yaml

입력:

최종 local waypoints (globally or locally planned path)

목표 속도(섹터 스케일 + 내부 로직)

출력:

VESC에 보낼 steering_angle, accel/brake 명령

###5.2 TinyLidarNet 제어 출력

TLN은 Planning + Control을 합친 E2E 구조지만,

Autoware 관점에선 “ML Planner가 바로 control 명령을 내는 특수 Control 모드”로 볼 수 있음.


## 6. Localization / Map / Perception / System 옆 박스 대응

다이어그램 왼쪽의 입력 박스들도 ForzaETH에서 이렇게 対응:

Map Component

 -  slam_toolbox로 만든 맵 + particle_filter/maps + stack_master/maps 디렉토리들

Localization Component

 -  particle_filter (SynPF), ekf_filter_node 조합

Perception Component

 -  /scan 기반 obstacle detection (Detect.cpp) + opponent tracking

System Component

 -  stack_master의 base_system, state machine, 시뮬/실차 스위치, 컨테이너 스크립트 등.




Human Machine Interface

조이스틱, RViz raceline editor, rqt dynamic reconfigure, 섹터 편집 GUI 등이 해당.
