# lanelet2

## 논문

### I. introduction

> Lanelet2: A high-definition map framework for the future of automated driving

- accuracy, completeness, verifiability and extensibility가 중요하다~
- HD map은 자율주행의 필연적이다. 센서는 완벽하지 않고, 센서로 측정 못하는 region의 정보를 지도가 전달한다.
- 또한, 센서의 불확실성을 고려해 관측된 데이터를 잘 '해석'해야 함도 있다.
- 우리는 이 논문에서 **lane-level accurate map** 라는 용어로 이걸 사용하겠다.
- highly automated drivin (HAD) maps

### HAD map과 어떤 상호작용이 필요한가요?

= figure 1

- direct access to elements
    - sensor simulation
    - localization
    - map validation
- lane and envrionmental layout
    - algorithm validation
    - scene understanding
    - special maneuvers
    - path planning
- road network
    - behavior generation
    - routing
    - object prediction

### II. related works

#### A. 기존 맵 포맷과 프레임워크

- tomtom, openstreemap 등이 있다. 기존 것들은 top-down 접근을 많이 취함.
- 네비게이션처럼, 우선 차도를 깔고, 그리고 차선은 속성 정보로 추가해주는 방식이다.
> 이것은 복잡성, 정보의 암시적 표현법을 야기한다.
> 그리하여 실제 그 차선의 절대좌표는, 중앙선에서 상대적으로 구해질 뿐이다.
> 특히 교차로에서 그 복잡도는 증대된다.
> 게다가 osm의 경우 데이터 퀄리티도 크게 상이하다.

- 이것에 대해 OpenDRIVE 프레임워크는, 거기에 구체적인 차선, 교차로 등을 입힐 수 있는 무료 편집기이다.
    - road ref line + basic shape + lane (xml)
    - +speed limit sign
    - 그러나 이걸 해석하는 무료 라이브러리가 없다?
- Unified Map
- Towards a Multi-hypothesis Road Representation for Automated Driving
- Vehicle Self-localization with High-Precision Digital Map
    - 앞서 말한것들의 단점과 달리 이거는 HAD map, lidar 기준 랜드마크 정보 기하학?까지 표현할 수 있다?
    - 'outstanding'이라 표현함 논문에서
- liblanelet는 중앙선 기준 상대적으로 묘사되는 차선에 한계가 있다고 지적하는 듯.. 교차로 커버가 어렵다고 말함
    - 이건 좀 다른 접근법을 취한다. HAD를 위해 특별 제작되었으며, 특히 motion planning을 위해서
    - atomic lane sections (=lanelet)에 기반한다.
    - traffic rule은 이른바, 'regulatory elements'이라 부른다.
    - 약점은, '이미 알려진 루트'에 대해서만 디자인되어질 수 있다는 거다.
    - HAD map과 어떤 상호작용이 필요할까요? 파트에서 소개한 모든 걸 구현하진 못했다.
    - 그 약점을 제거하기 위해 lanelet2를 알아보자


#### usage of maps

지도에 어떤 요소가 필요한지 알기 위해 지난 5년 간의 32 출간을 살펴보았다.

- 13 : 측위
- 5 : 맵 생성
- 4 : 시스템 아키텍처
- 4 : 모션 플래닝
- 4 : prediction
- 1 : scene understanding
- 1 : behavior generation and detection of map deviations에 대해 각각 다루시더라

- 그들중 7은 osm 포맷을 사용했고, 확장을 하였다.
- 그 중 5가지 접근 중 liblanelet의 사용이 있었다.

### III. Design considerations

figure 1에서 알아봤든, 앱이 집중할 요소는 크게 3가지로 나눌 수 있음.

#### A. road network

- routing : HAD는 더 높은 정확도 요구
    - 단순 전역경로 끝~이 아니라, 어떤 차선이 이용 가능한지, 버스 및 다른 전용차선이 있는지 등을 명확하게 파악해야 함
    - 이 의미론을 토대로, 차선 변경이 일시적으로 금지되거나, 막힌 경우 대체 경로를 찾을 디테일이 필요하다
- behavior generation
    - 추월, 끼어들기, 브레이킹, 신호대기하기 등
    - 트래픽 규칙에 'consistent'하게 대응되는 행동 집합을 생성하기 위해..
- prediction : 다른 도로 사용자를 예측해야한다~
    - 왠만해서는 차선 유지로 가정하고, 또는 트래픽 규칙에 의해서만 차선 변경을 한다고 행복한 상상을 할 수가 있겠다.
    - behavior generation과 비슷하게, 가능한 행동집합이 결정되야 한다.
    - 하지만 사용자 유형을 고려해야한다. ('버스'에게 특별히 적용되는 규칙이라던지..)
    - 그래서 단순 에고 차량 뿐만 아닌 다른 유형 차량의 트래픽 규칙도 포함해야함
    - 또한 특수 보행자, 자전거, 트램 등까지

#### B. land and environment

- path planning : 정확한 경로 설계를 위해, lane 기하학 정보도 정확해야 함.
    - 단순 중앙선만 아는 건 불충분
    - 곡선 구간에 기하학 정보에 따라, 궤적과 속도를 고려해야하기 때문이다?
- special maneuver
    - 종종 잊는 문제는, AV가 공공도로에서의 일반적 운전 외에도, 특별한 기동도 포함한다는거 (maneuver=기동)
    - 주차하는거나,
    - 다른 차량을 피해야 하거나, 센서 고장 후 안전한 위치에 도달해야 하는 등의 비상 상황
    - 도로에서 이용할 수 있는 풍부한 디테일 정보까지 줘야할 것임..

#### C. physical elements

- localization
    - 측위가 중요하기 때문에, 애초에 HAD 지도에 가능한 한 많은 센서가 관찰할 수 있는 요소 (랜드마크 같은거..)가 포함되어야 하며, 다양한 센서 설정을 갖춘 차량도 이용할 수 있어야 한다.
    - 이를 통해 '정확한 측위'가 가능토록 지원해줘야 한다.
    - marking, crash arrier, roadside 등
        - 게다가 이들은 도로에 흔한 기본 컴포넌트니까.. 명시하면 좋다~
- 주된 도전과제 : up to date이니? (지역적 변화에 반응하니?)
    - 디테일한 정보를 맵에 포함할 수록, 업데이트 해야 하는 야도 많아짐
    - 차선에 그냥 속도제한만 달아두고, 어느 표지판에 의함이지 출처 명시 안하면, 업데이트 반영이 어렵다.
    - 출처도 명시해야 한다.
- change detection
    - road의 structural change는 change detection 알고리즘에게 탐지 되어져야 한다
    - 애초에 중앙 집중식이 아니라, 실제 우리 네트워크 클라우드 군 차량 하나라도, 그걸 발견해서 서버에 업데이트 요청을 역으로 날려주면 되는 것 아니겠는가?
- 또 다른 챌린지 : 시간 경과에 따른 지도의 정확성 뭘로 보장할래?
    - 대륙 이동과 같은 환경 변화에도 안정적으로 저장되고 참조되어야 ㅋㅋㅋ
    - GPS 같은 전역 좌표 대신, 지역적으로 고정된 기준 프레임을 사용하는 게 필요한 것이다..
    - 생각하면.. 고정밀도를 위한다면 국지적 프레임이 맞긴 함..

#### D. consequense for maps

방금 챕터에서 말한 detectable elements를 *physical layer*라고 부를 것이다.

- 이것은 실제, 관측 가능한 요소들로 구성된다.
- 반면 차선과 같은 다른 모든 요소는 이러한 물리요소들을 더 추상적인 표현으로 '연결'한 의미론일 뿐이다.
- 이 후자를 *relational layer*라고 부르자.
- 이 이분화의 장점은, 이 relationl layer 같이 둥둥 떠있는 개념들이 실제

- 요약하자면, 우리는 다음 원칙들이 HAD를 위한 향후 HD 맵 포맷에 필수적이라고 생각합니다: