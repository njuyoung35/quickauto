# quick-start

rosbag : logged and time-stamped messages of topics

function provided by autoware:

- 측위
- 객체 감지
- driving control
- 3d map generation and sharing

- recognition : 객체 감지, 측위
- judgement : lane following, intersection <- autoware rider(app), autoware route, route planning
- operation : acc, break, steering -> vehicle control PC -> vehicle (CAN)

user interfaces:
- runtime manager, Rviz
- autoware rider, oculus, ..

# 시작

- replay of demo data
- autoware on a real vehicle
- rosbag replay

# rosbag을 사용하자

- 데이터를 아예 생성해놓고 돌린다..?

# 잡생각

- ak47같은 좀비성. 부품이 떨어져 나가도 센서 퓨전 등 통해 극복
- 

# autoware.auto

ros1 autoware.ai의 계승자. 
- 모던 소프트웨어 공학 : code review, CI testing, 문서화, test coverage, style & dev guide
- 라이브러리, 노드, 시스템 레벨에서의 재현성과 결정론성 강조

## AVP(valet parking)

`operation design domain` 사용

## behavior planner

# autoware.auto.design

- common
- control
- drivers
- fusion
- localization
- mapping
- perceptino
- prediction
- tools
- URDF

## 센서 회사들..

- autoware documentaiton/reference HW 확인~

- tier IV
- robosense

- driver : AV sw(autoware)와 특정 하드웨어를 연결해주는 sw 모듈 또는 인터페이스를 말함! (bridge)
- autoware가 이해할 수 있는 포맷으로 바꿔서 pub해줌.

### bridge 노드화의 장단점

- 단점: IPC, 딜레이 발생의 필연성
    - 직렬화 -> 소켓 통신 -> 역직렬화
    - 반면, 모놀리식 함수 호출은 단순한 포인터 접근
- 고성?능 QoS

```
sensor_qos:
  reliability: best_effort  # 혹시나 손실된 데이터보다 실시간성이 중요
  durability: volatile      # 과거 데이터는 필요없음
  history: keep_last       # 최신 데이터만 유지
  depth: 1                 # 버퍼를 최소화하여 레이턴시 감소
  deadline: 10ms           # 최대 허용 지연 시간
```

- 장점 : 생태계와 유지보수
    - 자율주행은 단순 빠름 뿐만 아니라, 안전하고 견고하며 합리적이고 발전 가능한 시스템이어야 하기 때문!

- autoware에서의 접근법
    - 고성능이 필요한 코어 드라이버
    - 로직과 드라이버의 분리
        - 드라이버는 간단한 변환 정도.. raw_data -> PC2
        - 처리, 로직 노드는 무거울 수 있음..
    - 성능 최적화? zero-copy, shared memory, RTPS, 리눅스 커널 설정?

### 센서 대략적으로

- AD(autonomous driving) Computer
- LiDARs
- Radars
- Cameras
- Thermo cameras
- IMU, AHRS(roll, pitch, yaw, heading 측정), GNSS/INS
- wire supply & network
- vehicle platform (차체)
- 조종기

# 데이터셋

## 이스탄불 오픈 데이터셋

- 다리, 터널, viaduct, road junction, 고속도로, dense urban areas 등

- leo drive - mapping kit sensor data
    - GNSS/INS
    - LiDAR

## Bus-ODD datasets

- leo drive - ISUZU sensor data

# operational design domain

ADS=autonomous driving system

- SAE J3016 : 자율주행 0~5단계 정의 (taxonomy)
- ODD : ADS의 특정 작동 조건을 정의하기 위한 운행설계범위
- DDT : dynamic driving task
- OEDR : object and event detection and response
- ODD를 설정해 문서화하고 ADS 기능평가, 시험, 검증 절차를 정의하고 문서화하자

## 세종시 셔틀버스 자율주행 테스트

http://journal.ksae.org/xml/25348/25348.pdf

- 실증구간 교통량 분석 - ITS

## ODD의 구성요소

- physical infrastructure
    - road type : main arterial road, signal intersection, non-signal intersection, roundabouts, crosswalk (bicycle crosswalk), school zone
    - roadway surfaces : pot holes
    - roadway edges & markings : lane markings, lane obstacles (lane control bars)
    - roadway geometry
- operation constraints
    - operational speed limits : operation speed range(min~max)
    - traffic conditions : construction zones, accident zones
- objects
    - signage (간판) : traffic signs, traffic light signals
    - roadway users : vehicles, pedestrians, cyclists, motorcycles
    - none-roadway users : animals, debris
    - traffic equipment (고깔, 펜스, 공사중, 바리케이드, 거울 등등..) : CCTVs, traffic cameras
- environmental conditions
    - weather : rain, snow, fog
    - weather-induced road conditions : road flooding/snowing/freezing
    - illumination : night
- zone
    - traffic management zone : traffic signals by people (traffic police, workers, etc)
    - school zone : vernerable user caution
    - construction zone : caution of road installations and heavy equipment
    - interference zone : dense area of high-rise buildings
        - 고층 건물로 인해 GPS 신호 불안함
- connectivity
    - demenstration vehicle : V2X communication
    - infrastructure sensors : work area warning, operation route and accident management, pre-detection and avoidance of risk factors, collection of traffic information
    - digital infrastructure : HD maps, V2X networks (WAVE, LTE, 5G)

이 테이블에 대하여, 당신의 시스템이 운행 가능한 조건(ODD)는 뭔지 명시하고, 또한 운행 가능하지 않은 조건 (limit / boundaries)를 기술하도록 해라

- 이 체크리스트는 아마도.. level 3이상의 자율주행 '기능'을 위한 것으로 보인다.

V2? 용어
- V2X : vehicle-to-everything
- V2D : vehicle-to-device
- V2G : vehicle-to-grid (smart grid)
- V2N : vehicle-to-network
    - V2C : vehicle-to-cloud
    - V2I : vehicle-to-infrastructure
    - V2P : vehicle-to-pedestrian
    - V2V : vehicle-to-vehicle

- OEDR의 구성요소
    - OEDR은 자율주행차가 달리는 주행환경의 구성요소를 중심으로, 객체와 그러한 객체로부터 발생 가능한 상황 및 event에 따른 ADS의 대응을 구체적으로 '서술'하는 방식으로 작성될 수 있다.
    - 각 객체와 각 객체로부터 발생되는 상황 및 event에 대해 ADS는 detection 및 response를 수행하게 된다.

충북대 오창캠퍼스 자율주행차 테스트베드 C-track
    - Euro NCAP 2025 로드맵에 따른 AEB, LKAS, ACC 등 ADAS 기능평가, 친환경차 중심의 미래형 자동차 성능 검증 시스템을 구축할 예정