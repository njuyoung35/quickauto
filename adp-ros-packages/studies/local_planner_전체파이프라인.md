우리가 만들 통합 구조 한 줄 요약

- Autoware 쪽

  * Lanelet2 기반 Vector Map(차선 중심선, 경계, shoulder 등) 로드

  * Behavior Path Planner의 Lane Following 모듈이 lanelet centerline을 참조해서 기본 reference path 생성 

- 중간 어댑터

  * Vector Map + Autoware reference path → ForzaETH가 이해하는 global_waypoints + d_left/d_right 형식으로 변환
  * 즉, mapping_launch.xml → global_planner → global_waypoints.json 생성하는 기존방식을 >>> **Vector Map Adapter → global_waypoints_autoware.json** 이렇게

- ForzaETH 쪽

  * 기존처럼 Particle Filter, Controller는 유지

  * **Spliner(회피 경로 생성)**가 “Teras occupancy-map 경계” 대신 “Lanelet 기반 차선 경계/shoulder 폭”을 사용해서 회피 경로 생성하도록 수정함
 
---

      [Autoware Vector Map (Lanelet2)]
        └─ lane centerline, left/right bounds, lane connection, shoulder 등
      
             ▼
      
      [Autoware Path (Lane Following)]
        └─ lanelet centerline 기반 reference path 생성 (Behavior Path Planner – Lane Following) :c공ontentReference[oaicite:2]{index=2}
      
             ▼ (topic 예: /planning/trajectory 혹은 /planning/path)
      
      [Vector Map Adapter (새로 만드는 패키지)]
        ├─ Autoware reference path + lanelet geometry
        └─ ForzaETH 형식의 global_waypoints.json equivalent 생성
            · s, x, y, yaw
            · d_left, d_right (= lane_width, shoulder 폭 등)
            · speed, curvature…
      
             ▼
      
      [ForzaETH Race Stack]
        ├─ (옵션) Global Planner 건너뛰고, Adapter가 만든 global_waypoints 사용
        ├─ SynPF / EKF → ego pose, frenet 좌표 생성 
        ├─ Opponent Estimation(Detect + Tracking) → 장애물/상대차 s, d, vs, vd :contentReference[oaicite:4]{index=4}
        ├─ State Machine (GBTRACK / TRAILING / OVERTAKE / SPLINE…) :contentReference[oaicite:5]{index=5}
        └─ Spliner (spline_planner) → Vector Map 기반 경계/폭을 사용해 회피 경로 생성 
      
             ▼
      
      [L1 / MAP Controller]
        └─ local path + target velocity → steer, accel, brake 생성 → Autoware 시뮬 차량에 적용 :contentReference[oaicite:7]{index=7}

---

## 2. autoware측

### 2-1. Vector Map Loader (Lanelet2)
- Lanelet2(.osm) 기반 Vector Map 로드
- 차선 중앙선, 차선 경계, 속도 제한, 차선 연결 등을 제공

      Lanelet2 Vector Map (.osm)
          ├─ 차선 geometry (centerline)
          ├─ left/right bound
          ├─ shoulder, parking, 교차로 link 등
          └─ traffic rule (speed limit …)
      
             ▼
      
      [Autoware Map Server / lanelet2_map_provider]
        └─ Autoware 내부에서 /map/* 형태로 제공


### 2-2. Behavior Path Planner – Lane Following

- lanelet centerline을 따라 기본 reference path 생성
- 우리 쪽에서는 이걸 **Forza global_waypoints의 “초기 레이스라인”**으로 재활용 가능

        입력
        ├─ Vector Map(lanelets)
        ├─ Localization (ego pose)
        └─ Goal (API Layer에서 오는 목표 지점) :contentReference[oaicite:10]{index=10}
      
             ▼
      
      Lane Following 모듈
        ├─ lanelet centerline 추출
        ├─ route 상 lanelet들을 연결해 연속 path 생성
        ├─ 곡률/경계 고려하여 smooth path
        └─ (기초 속도 프로파일도 포함 가능)
      
             ▼
      
      출력
        └─ /planning/path 또는 /planning/trajectory
           (pose, velocity, acceleration 시퀀스)

==> **우리가 할 일: 이 Autoware path + Vector Map을 받아서 “Forza 방식 global_waypoints 구조”로 변환하는 어댑터 작성.**


## 3. Vector Map Adapter 패키지 흐름도 (새로 설계하는 핵심적인 부분)

### 3-1. 입력

Autoware 측:
- /planning/path or /planning/trajectory
- lanelet2 map (서비스/토픽 혹은 Lanelet2 API)

### 3-2. 처리 과정

      [1] Autoware Path 샘플링
        - Autoware path에서 일정 간격(s_step)으로 점 샘플링
        - 각 점에 대해:
            · x, y, yaw
            · 곡률(curvature) 계산
      
      [2] 해당 점이 속한 lanelet 찾기
        - lanelet2 쿼리:
            · closest lanelet
            · left bound / right bound geometry
      
      [3] d_left / d_right 계산
        - centerline 기준 왼쪽 경계까지의 거리 → d_left
        - 오른쪽 경계까지의 거리 → d_right
        - shoulder/parking lane까지 포함할지 정책 결정
          (예: 회피 시 shoulder까지 허용이면 d_left/d_right 더 크게)
      
      [4] 속도 프로파일 생성
        - lanelet 속도 제한
        - 곡률 기반 최대 속도
        - 섹터 비슷한 개념이 필요하면 Autoware route를
          규칙적으로 s 구간으로 쪼개서 sector_id 생성
      
      [5] Forza 글로벌 웨이포인트 포맷으로 변환
        - race_stack에서 사용하는 구조를 그대로 맞춤:
          · s
          · x, y, yaw
          · d_left, d_right
          · curvature
          · v_ref (목표 속도)
          · sector_id(optional)

### 3-3. 출력

- global_waypoints_autoware.json (forza 포맷)
- ROS 토픽으로도 발행 가능:
  * /global_waypoints (nav_msgs/Path)
  * /global_waypoints_scaled (Pose + 속도 포함)
- 파일 위치/포맷은 기존 race_stack이 기대하는 위치/형식을 따름

==> 이렇게 만들어 두면, ForzaETH global planner(mincurv/mintime)를 아예 안 돌리고, “Autoware 벡터지도 기반 레이스라인”을 바로 쓸 수 있을 것.


## 4. ForzaETH 쪽 패키지 흐름도 (회피 경로 관점으로만)

**“어디를 고쳐야 회피 경로가 벡터지도 기반이 되는지”**에 집중함.

### 4-1. 현재 ForzaETH 회피 경로 파이프라인 (Forza 매뉴얼)

      [PF / SynPF]
        └─ ego pose in map frame
      
      [State Estimation + Frenet Odom]
        └─ ego (s, d, yaw_frenet) 계산
      
      [Global Planner]
        ├─ occupany map에서 좌우 경계(bound_left/right) 추출
        ├─ mincurv/mintime로 global_waypoints.json 생성
        └─ 각 wp의 d_left, d_right, curvature, v_ref 포함
      
      [Opponent Estimation]
        └─ 장애물/상대차의 s, d, vs, vd
      
      [State Machine]
        └─ GBTRACK / TRAILING / OVERTAKE / SPLINE 등 모드 결정
      
      [Spliner (spline_planner)]
        - 입력: global_waypoints + d_left/d_right + opponent
        - 출력: /planner/avoidance/otwpnts (Spline 회피 경로)
      
      [Controller (L1/MAP)]
        └─ local path 따라가며 steering/accel 출력

### 4-2. 벡터지도 기반으로 바꿀 때 수정할 부분

**Spliner가 참조하는 “트랙 경계 정보”**를 “Autoware Vector Map → Adapter에서 계산한 d_left/d_right”로 바꾸는 것

* (1) Global Planner 단계

      옵션 2개:
      a. Autoware 기반 global_waypoints를 그대로 쓰기
      - Global Planner(mincurv/mintime)를 건너뜀
      - Adapter가 만든 global_waypoints_autoware.json을
      - Forza의 readwrite_global_waypoints.py에서 기본 파일로 사용하도록 변경
      b. Autoware global path를 mincurv/mintime 초기 guess로 넣고, Forza global planner로 한 번 더 최적화
      
      지금 목표는 *“벡터지도 정보를 쓰는 회피 경로”*니까, 1번으로 단순화하는 게 맞고, 나중에 시간이 남으면 2번을 추가하면 될 듯.

* (2) Spliner / spline_planner 수정 포인트

      Spliner가 쓰는 주요 입력:
            gb_wpnts.d_left, gb_wpnts.d_right
            spline_bound_mindist
            evasion_dist
            obs_traj_tresh
      
      현재는 이 d_left/d_right가 occupancy map → global planner에서 나온 값인데,
      앞으로는 Adapter가 계산한 lanelet 기반 d_left/d_right가 들어오게 됨.

      기존:
        - 트랙 경계: occupancy grid 경계에서 추출
        - d_left/d_right: 차체 중심에서 경계까지의 거리 (실험적으로 수동 튜닝)
      
      변경 후:
        - 트랙 경계: lanelet left/right bound
        - d_left/d_right:
            = lanelet centerline에서 왼쪽/오른쪽 경계까지의 거리
            (+ 필요하면 마진을 더해서 safety_width처럼 사용)
        - shoulder까지 포함해서 더 넓은 공간을 회피에 쓰고 싶으면
          lanelet 상의 shoulder polygon까지 사용해서 거리 계산


  ---

## 필수 수정 지점(예상)

spline_planner.py (또는 spliner의 core 로직)에서:

gb_wpnts.d_left, gb_wpnts.d_right를 읽는 부분은 그대로 두되,
이 값이 “벡터지도 기반”으로 들어오도록 global_waypoints 생성 쪽만 바꾸기

* spline_bound_mindist 설계
 - lanelet 폭이 일정치 않다면, spline_bound_mindist를 relative ratio로 설계하는 것도 고려
  - (예: lane_width * 0.2 이런 식)

* evasion_dist
 - 차선 폭과 shoulder 여부에 따라
  · lane안에서만 회피할지
  · 옆 차선/shoulder까지 쓸지 정책 결정

* 장애물 위치 계산
  - 장애물 s, d 계산은 기존 opponent tracking 그대로 사용 가능
  - 다만 Autoware perception(/detected_objects)을 쓰고 싶으면 이걸 Forza의 opponent 입력 구조(s, d, vs, vd)에 맞게 변환하는 “perception adapter”를 하나 더 만들면 됨.

---

코드 레벨에서 손대야 할 부분을 요약 : **Adapter 설계 + Spliner 튜닝**

* Vector Map Adapter 패키지 (새로 작성)

   입력: Autoware /planning/path + lanelet2 map
   
   출력: Forza 스타일 global_waypoints_autoware.json + /global_waypoints(_scaled)
   
   d_left/d_right, sector, v_ref까지 계산

* Forza global_waypoints 로더 쪽 수정

   readwrite_global_waypoints.py에서 기본 사용 파일을
   global_waypoints_autoware.json으로 선택하도록 변경

* Spliner(또는 spline_planner) 확인

   d_left/d_right, spline_bound_mindist, evasion_dist 사용하는 부분이
   “맵 기반 경계”라는 가정에 맞게 동작하는지 확인
   
   lanelet 기반 d_left/d_right 값의 범위를 보고 파라미터 재튜닝 (예: spline_bound_mindist, obs_traj_tresh)

* Perception Adapter  <<이건 옵
   
   Autoware /perception/object_recognition/objects →
   Forza Opponent Estimation이 기대하는 입력 포맷(s, d, vs, vd)으로 변환하는 노드 추가
