## **ForzaETH가 occupancy map에서 얻던 정보를 Autoware Vector Map에서 뽑아주기 위한 Adapter Node 만들기**
Adapter가 반드시 만들어서 race_stack(특히 Spliner + Controller)이 필요로 하는 자료는 global_waypoints.json과 동일한 구조의 JSON이다.

Adapter노드 : Autoware의 출력(Path / Vector Map 정보)을 ForzaETH race_stack이 이해할 수 있는 global_waypoints 형식으로 바꿔주는 ros2 변환기 노드

---
* Adapter가 수행할 기능

      - Autoware의 /planning/path or /planning/trajectory 구독
      
      - Lanelet2 맵에서 현재 경로가 속한 lanelet 추출 (Lanelet(lanelet2) 은 도로를 차선 단위로 표현한 공식적인 표준 데이터 구조)
      
      - 각 waypoint마다:
      
            x, y, yaw
            곡률 curvature
            d_left, d_right 계산
            (lanelet centerline 기준 왼/오른쪽 경계까지 거리)
      
      - 속도 프로파일 v_ref 생성
      
      - lanelet speed limit
      
      - curvature 기반 속도 제한
      
      - ForzaETH race_stack이 요구하는 global_waypoints.json 형식으로 변환 >> 이걸 /global_waypoints /global_waypoints_scaled 토픽으로 publish

* 입력(Input)
      (1) Autoware Path 메시지
      
      /planning/path 또는 /planning/trajectory
      
      메시지 타입:
      
      autoware_auto_planning_msgs::msg::Path
      
      또는 Trajectory
      
      내용:
      
      pose (x, y, z, quaternion)
      
      curvature(optional)
      
      target_speed(optional)
      
      time_from_start(optional)
      
      (2) Lanelet2 Map
      
      lanelet2_map (Autoware map_server에서 로드)
      
      Python이면 lanelet2 Python API, C++이면 lanelet2_core 사용

* 출력(Output)
      (1) ForzaETH global_waypoints.json equivalent
      
      (파일 저장 + 토픽 publish)
      
      각 waypoint 구조 (기존 race_stack 형식 그대로)
            {
              "s"        : float,
              "x"        : float,
              "y"        : float,
              "yaw"      : float,
              "curvature": float,
              "d_left"   : float,
              "d_right"  : float,
              "v_ref"    : float
            }
      (2) ROS2 토픽 출력
      
      /global_waypoints → nav_msgs/Path
      
      /global_waypoints_scaled → nav_msgs/Path (velocity 포함)
      
      /adapter_info (diagnostic optional)
---

==> 결론: Spliner는 많은 파라미터에 의존한다 > Spliner를 그대로 사용 가능하게 하려면 global_waypoints에서 반드시 맞춰줘야 하는 항목들이 많다.

# 1. Spliner가 실제로 사용하는 입력들 전체 목록

참고 자료:
* **spliner_README.md**  
* **ForzaETH race stack manual**  

Spliner의 입력은 크게 4계층 :

---

## 1) global_waypoints (전역 경로 전체 정보)

### Spliner가 여기서 사용하는 필드:

* x, y
* yaw
* curvature
* s (프레넷 좌표 변환에서 사용)
* **d_left, d_right**
* v_ref (목표속도, pre/post apex 거리 스케일링에 영향)

간접적으로 중요:

* 레이스라인 곡률 변동
  → apex 위치 계산에 영향
* waypoint 간 간격
  → spline 샘플링 품질에 영향
* 경계(boundary) 형태
  → 회피 가능 여부 결정

즉, global_waypoints 전체 품질이 Spliner 동작 품질을 결정한다. 

---

## 2) frenet odom 변환 (ego state)

Spliner는 반드시 ego state를 **Frenet(s, d, yaw)** 좌표로 받는다:

입력:

* /car_state/odom_frenet
  (SynPF → frenet_republisher)

frenet 변환은 global_waypoints의 품질에 아주 민감함:

* waypoint의 s 증가가 매끄럽지 않으면
  → s_lookup error 발생
* 곡률이 튀면
  → yaw_frenet 오차 발생
* waypoint 간격이 불균일하면
  → d offset 계산이 튐

==> 그래서 Autoware path를 그대로 Spliner에 넣으면 안 되고, Adapter에서 **균일한 s 간격으로 재샘플링**해야함.

---

## 3) Spliner 내 파라미터 그룹 (핵심)

spliner_README 기준 Spliner 파라미터:  

### A. 회피 여부 판단 관련

* `obs_traj_tresh`
* `spline_bound_mindist`
==> 경계 거리(d_left/d_right), 장애물과 경로의 상대 거리(s, d)에 영향.
==> Autoware 벡터지도 기반 도로는 폭이 일정하지 않을 수 있나? → threshold 재튜닝?

---

### B. Spline Control Points (apex 계산)

* `pre_apex_0`
* `pre_apex_1`
* `pre_apex_2`
* `post_apex_0`
* `post_apex_1`
* `post_apex_2`

**거리가 아니라 global_waypoints의 resolution·곡률에 맞춰 스케일링되어야 함.**
즉, global_waypoints를 Autoware path로 바꾸면 높은 확률로,

* waypoint 간격이 다름
* 곡률이 다름
* s-progress 방식이 다름

따라서,
→ apex 주변 control point 계산이 모두 달라진다.
→ control points 값을 그냥 그대로 쓰면 spline의 모양이 깨질 수 있음.

Autoware path에 맞춰 **pre/post apex distance를 새로 튜닝**.

---

### C. 장애물 예측 관련

* `kd_obs_pred`
* `fixed_pred_time`

장애물 예측은 **opponent Tracking 출력(s, d, vs, vd)** 기반인데,
vector map 기반 도로는 폭이 Autoware occupancy-track과 다르다.

즉,
* d 좌표의 기준점이 바뀐다
* shoulder나 방향구분선이 있을 수 있다
* s 증가율도 달라진다

→ 장애물 예측 속도 및 진행 방향의 좌표계 의미가 달라짐.
→ kd_obs_pred 및 pred_time도 튜닝 필요.

---

### D. track bound 관련

Spliner는 “track bounds”를 매우 중요하게 봄:  

* global_waypoints.d_left
* global_waypoints.d_right
* * spline_bound_mindist

의 조합으로 경계 충돌 여부를 판단하는 것 같음.

Autoware lanelet 기반 경계는:

* 직선 구간 폭 3.5m
* 곡선에서는 폭이 변하거나
* shoulder, parking으로 확장될 수 있음

→ 기존 Teras occupancy map(점유격자지도) 기반 폭과 완전히 다름.
      ==> 기존 Teras/Ouster 기반 occupancy map은:
      
      벽이 울퉁불퉁하거나
      
      코너에서 안쪽/바깥쪽 벽이 곡률 때문에 차이나고
      
      손수 편집(GIMP)하면서 약간 비뚤어지기도 하고
      
      그래서 도로 폭이 1~2m 차이로 여기저기 달라지는 건 정상이었음.

      하지만 트랙은 “단일 레이싱 트랙”
      경계는 “벽(Obstacle)” >> 의미는 “여긴 못 들어감”
      의미: “여기서부터는 충돌”
      복수 차선도 없고, shoulder도 없음

      ==> Autoware lanelet map은 “도로 구조 전체”를 담고 있어서:
      
      lane width (차선 폭)
      shoulder (갓길)
      parking space
      walkway
      merging/diverging lane
      median (중앙분리대)
      bus lane
      → 이런 구조들이 경계(leftBound/rightBound) 안에 그대로 들어감.
      예를들어, lanelet leftBound = 차선 경계, rightBound = shoulder까지 포함될 수 있음
      
      **그래서 폭 변화는 단순한 벽 위치 변화가 아니라 기능이 다른 경계들 때문에 생기는 폭 변화임.**

그래서:

* d_left/d_right
* * spline_bound_mindist
    둘 다 Autoware vector map에 맞춰 **동시에 튜닝**해야 한다.

---

### E. 그 외

* 장애물 좌표 기준이 전혀 다름 (lanelet 기반 vs occupancy 기반)
* Spliner 내부에서 y offset 계산 시 centerline의 yaw 변화가 중요
  → Autoware path가 거의 직선 위주면 예외상황 발생 가능
* Forza는 레이싱라인이 곡률이 크고 복잡함
  Autoware centerline은 훨씬 smooth → control point 분포 변경 필요

---

*d_left/d_right
* evasion_dist 적용
* spline_bound_mindist 체크
* 경계 collision 체크
* 회피 방향 선택 (왼/오른쪽)
* apex 위치 조절

---

# Spliner가 Autoware 기반에서 제대로 돌아가려면 **아래 그룹 튜닝 필수.**

| 그룹 | 항목                         | 필요성                                 |
| -- | -------------------------- | ----------------------------------- |
| 1  | d_left / d_right           | 경계 기반 판단 로직 전체가 의존                  |
| 2  | s 및 waypoint 간격            | Frenet 변환 품질 좌우                     |
| 3  | curvature                  | 속도와 apex control point spacing 영향   |
| 4  | pre/post apex 파라미터         | Autoware path resolution에 맞게 재튜닝 필요 |
| 5  | 장애물 예측 파라미터(kd_obs_pred 등) | lanelet 기반의 d 의미가 달라져 조정 필요         |

---

## Spliner가 쓰는 입력 전체(global_waypoints + ego + opponent + parameters)를
## Autoware 기반으로 모두 재정렬해야.

---
