link : https://dgist-artiv.github.io/category/hdmap.html

# 액기스

- Euro NCAP AD Test and Assessment Protocol v2.0
    - 자동차 안전 평가 프로그램. 어떤 시나리오가 있는지 공부할 만 함.

# DGIST ARTIV Lab.

- 대구경북과학기술원 학부생 도전과제 UGRP 수행팀 ARTIV 입니다.
- 연구 과제명 : 자율주행자동차 기반 스마트 캠퍼스 조성

## main research topic

- Autonomous Vehicle Development
- ADAS Sensor application development
- Integrated SW architecture development and research
- Planning/Control alogorithm for Autonomous Vehicle
- HD Map and Integrated Localization Algorithm with SLAM and GNSS

실제 ARTIV Shuttle이 있는 걸로 보임. sensor layout, ext sensor layout 등 명세가 나와 있음.

## 산하조직

- LIDAR/RADAR Part
- Computer Vision Part
- HDMap/Localization Part
- HW/Communication Part
- Control/Planning Part
- integrated SW Part

# LIDAR/RADAR

## forward collision assist test

- https://dgist-artiv.github.io/lidar/2020/08/29/fcw-test.html
- 전방 충돌 방지 보조(FCA) 알고리즘
- 센서의 사각지대에서 갑자기 튀어나오는 imminent threat에 대응하는 난제
- 핵심 목표 : 정확히 인지된 위협을 적절한 개입으로 사고를 방지한다.
- 무작정 급정거시 2차 사고 위험 있음. -> 신중히 동작하는 로직이어야.
- Euro Ncap 혹은 여타 안전 평가 기관에서 제시하는 Inter Urban에서의 ADAS 시스템의 안전 기준에 맞춰 동작하도록

### 어떻게 반응할까?

일단 세 가지 정도 카테고리로 나뉘는 걸로 보입니다

- ACC BREAKING : 감속
- EMERGENCY BREAKING : 급정거
- VEHICLE CUTTING_OUT? (자세하지 않음) : 회피

## lidar segmentation & tracking

- https://dgist-artiv.github.io/lidar/2020/11/15/Lidar-Tracking.html
- 실제 구현 상 downsampling을 거쳐 lidar 데이터 중 일부만 사용했다고 함.
- 단순히 용어를 좀 알아가자면..
    - computer vision 분야에서 세그멘테이션 : 이미지의 각 픽셀을 특정 클래스로 분류하기. (나무에다가 '나무' 라벨 붙이기)
    - k-means
    - RoI(region of interest)
    - GT(ground truth)
    - kalman filter : 로봇 state 추정하기 위해서도 사용하지만 여기서는, roi를? tracking하기 위해 사용?

# Computer Vision

## Camera, Lens Selection and Setting

- 카메라 몇 개, FPS, 화질, fov?
- fov와 해상도에는 trade-off가 있다. (절대적 픽셀수를 고정해 본다면)
    - 카메라 센서는 수백만 ~ 수천만 개의 광감지 소자(포토다이오드)가 배열된 물리적 칩
    - 센서의 전체 픽셀 수는 그 하드웨어 상 고정이지만, 실제 이미지/비디오로 저장/처리될 때는 달라질 수 있음. 다운샘플링 or 픽셀 병합(binning) / crop or zoom

### 최종 구매

- FLIR Grasshopper3 USB3
- FLIR Blackfly S USB3
- ACH0518M10M (렌즈)

## ros2 + python3 + FLIR

- spinnaker sdk
- PySpin: FLIR official python library
- EasyPySpin: unofficial wrapper for FLIR Spinnaker SDK

```python
cap = EasyPySpin.VideoCapture(0)
cap.cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerGB8)
# 이제 아래와 같이 기존의 opencv와 비슷한 방식으로 사용할 수 있다.
ret, img = self.cap.read()
img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB)
img = cv2.resize(img, dsize = (args.width, args.height))
temp=CvBridge().cv2_to_imgmsg(img, encoding = 'bgr8')
self.publisher_.publish(temp)
```

이 코드는 FLIR_ImgPublisher라는 노드에서 FLIR_IMAGE라는 topic을 publish하는 구조가 된다.

- 웬만해서 대부분의 우리가 짜는 대부분의 코드는 NVIDIA Jetson 같은 중앙 허브에 저장되고 실행되는 바이너리가 된다.
- 과거에는 전통적/고가형인 분산식 (임베디드 게이트웨이) 아키텍처를 쓰기도 했다는데
- 현대 경량/저가형으로 중앙 집중식을 쓰는 경향이 있는 걸로 보인다.
    - 장점 : 저지연 통신, 시스템 간소화, 센서 퓨전 용이
    - 물론 모터 제어기 (BLDC), 고속 라이다 - 전용 처리 보드, 레이더 센서 - 신호 처리 모듈, CAN 통신 등 중간 임베디드가 필요한 경우가 있을 수도 있다. 특히 ADAS 같이 복잡한 센서체계를 구축할 경우..
- ros2의 노드 기반 아키텍처 철학은, 현실에서 NVIDIA Jetson 같은 중앙 허브에 모든 프로그램이 돌아가더라도, 그게 monolithic 프로그램이 아니라, 여러 논리적 프로세스 (node)로 분할되어 QoS 정책 등에 따라 돌아가는 추상화, 마법을 제공하는 것이다.

## lane annotation tool

https://dgist-artiv.github.io/vision/2020/06/24/Lane_annotation_tool.html

- 논문에서 평가용으로 많이 사용하고 있는 lane open dataset은 CULane, TuSimple, BDD100K이 있다
- 이 중 CULane 채택 (중국 홍콩대 데이터셋)
- 한국 도로 데이터도 CULane 데이터셋과 같은 라벨링, 어노테이션을 붙일 필요가 잇다. -> annotation tool 필요 (DGIST에서 직접 구현)

### 어노테이션 도구의 핵심 기능

- 동영상의 화면(이미지) 하나하나를 띄워주고 저장하는 기능
- 차선 4개를 구분짓는 기능
- 띄운 이미지에 마우스 클릭 시 x좌표 y좌표(ground truth가 될 지점)을 text파일에 쓰고 저장하는 기능
- annotation한 이미지들에 대한 segmentation image를 생성하고 저장하는 기능
- (별도의 코멘트) 회사에서 주문, 고객 요청, 명세에 맞추어 프로그래머가 프로그램과 서비스, 도구를 개발하듯이 저희가 팀플하는데 있어서 필요한 요구사항을 명확히 구체화, 언어화해서 서로 소통하는 자세가 중요할 것 같애요~

## Probabilistic lane estimation from ENet-SAD's result

- RANSAC (RANdom SAmple consensus) : 데이터셋에서 노이즈를 제거하고 모델을 예측하는 알고리즘
- 특정 임계값 이상의 데이터 완전 무시해버림 (outliner에 강건)
- Hypothesis 가설 단계 : 전체 데이터에서 N개 샘플 선택, 그걸 통해 모델 예측
- Verification 검증 단계 : 데이터셋에서 모델과 일치하는 데이터의 수 센 후, 최대 값일 경우 모델 파라미터 새롭게 저장
- 위 단계들을 N회 반복
- RANSAC의 핵심 : 데이터를 추출해 샘플이 대부분 inliner가 되도록 하는 것. 사용자가 종료 시점을 파악할 수 없는 알고리즘임.
- RANSAC의 파라미터 - 그래도 유한시간 내 종료시켜야 하기에
    - P  =  inlier로만 이루어진 샘플을 획득할 확률 – 샘플링 성공
    - α  =  dataset에서 inlier의 비율
    - m = 회당 추출하는 데이터 수
    - N  = 알고리즘 반복 회수
- RANSAC이 장단점도 있지만, computer vision 분야에서 잘 쓰인다고 합니다

## Depth Estimation with Monodepth2 using ROS

Monodepth2는 다이렉트로 depth map(센서 기준 사물까지의 거리 맵)을 내보내는 LiDAR, RGB-D 카메라 (intel realsense 등), 스테레오 카메라 (인간의 양안처럼, 두 개의 카메라를 이용해 계산) 같은 전문 센서 없이도, DNN을 이용해서 depth map을 뽑아내는 프로그램입니다.

- 단안 카메라로도 얻을 수 있으며, 자체적으로도 자가지도학습을 돌리는 것으로 보임
- 입력 : `/camera/image_raw`
- 출력 : `/camera/depth_map`
- 이거를 장애물 회피, SLAM, 객체 탐지 노드에서 LiDAR 데이터처럼 사용할 수 있는 것!!
    - LiDAR는 자체적으로 높은 정확도의 점군을 제공. 센서값, 비쌈.
    - Monodepth2 : 단일 카메라 이미지만으로도 depth map을 추정하는 소프트웨어 알고리즘.

## Side Occupancy Check

https://dgist-artiv.github.io/vision/2020/11/16/Side_Occupancy_Check.html

- DGIST 차량은 좌우 물체 여부를 숫자로 복잡하게 처리하기보다는 그냥 불리언 true/false로 판단하는 로직을 갖고 있는 것 같다.
- (코멘트) 우리가 가능하면 데이터가 상세하고 복잡해야 안전할 거라고 여기는 생각이 있는데, 우선 이런 간단한 불리언 데이터로도 돌아가는 로직, 프로토타입을 빠르게 찍어내는 게 중요하다는 생각이 든다.

## 차선 변경

- tracking algorithm에게 기존 차선 이미지에서 주행유도선을 그린 증강 데이터를 보내는 방법론은 어떠할까?
- 투박하고, 위험할 수 있다고 봄
- 정교한 시스템에서는 기피됨? - 인식, 제어 등 명확히 분리된 아키텍처 좋아보임
- 제어시스템 내의 guidance 역할.
- 개발/테스트 단계에서 변혀된 이미지를 사용해 알고리즘을 훈련하는 것은 매우 일반적이고 중요한 방법론.

## 주행기록계 DTG

- 자율주행차량 임시운행허가 받기 위해서는 주행기록계 설치해야.
- 자율주행자동차의 안전운행요건 및 시험운행 등에 관한 규정 제 17조 (운행기록장치 등) 자율주행자동차에는「교통안전법」제55조제1항에 따른 운행기록장치를 장착하여야 하고, 운행기록장치 또는 별도의 기록장치에 다음 각 호의 항목을 저장하여야 한다. 
- DGISTdptjsms ioniq 차량 사용중..

1. 자율주행시스템의 작동모드 확인 : /Ioniq_info/Auto Standby Switch(자율주행 대기 모드 on/off 여부)
2. 제동장치 및 가속제어장치의 조종장치 작동상태 : /Ioniq_info/APS Feedback(가속페달 신호) /Ioniq_info/BPS Feedback(브레이크 페달 신호)
3. 조향핸들 각도 : /Ioniq_info/Steering Angle
4. 자동변속장치 조종레버의 위치 : /Inoiq_info/Gear, Shift Position 같은 토픽 (기어 P/R/N/D)

- rosbag 외에도, 이를 쉽게 사용하게 해주는 래퍼? bagpy (윈도우)가 있다고 하는 듯.
- (코멘트) 프로젝트 구현 및 시연, 검증, 데이터 수집 등에 있어서 rosbag를 배우고 잘 사용해야할 것 같습니다.

# HDMap/Localization

## GNSS, 좌표계

- 수준점 : 국가기준점의 하나로, 평균해수면으로부터 정확하게 측정된 높이(표고) 값이 부여된 지점
- GNSS 센서 정확도 검증하기 위해 발로 뛰어볼 수 있다..

### 구글 지도에서 볼 수 있는 위도, 경도의 단위는 무엇이고 어떤 좌표계를 기준으로 할까?

- 좌표계 : WGS84 (타원체)
- 37° 33' 59.86" N
    - 도분초 (DMS) 사용. 위도의 경우 0~90, 경도의 경우 0~180 일 것으로 보임.
- 위도와 경도의 의미 (N, S, E, W)
- N/S : 북반구, 남반구
- E/W : 본초 자오선으로부터 동쪽, 본초 자오선 서쪽. (본초 자오선=영국 그리니치 천문대)

### WGS84와 EPSG 코드와의 관계.

- EPSG:4326 같은 인스턴스는, WGS84 좌표계의 고유 ID이다.
- proj4 : 좌표 변환을 수행하는 오픈소스 라이브러리. proj4 문자열은 좌표계의 모든 매개변수를 텍스트로 정의한 것.
    - `+proj=longlat` : 경도/위도 방식이다
    - `+proj=tmerc` : 횡단 메르카토르 도법
    - `+ellps=WGS84` : WGS84 타원체를 사용하겠다
    - `+datum=WGS84` : WGS84 기준계를 사용하겠다
    - `+lat_0=38 +lon_0=127` : 한국 중심 (38°N, 127°E)
        - 한국 중심으로 투영하여 왜곡 최소화
    - `+x_0=200000 +y_0=500000` : 한국만의 고유 좌표 원점
        - x, y : 특정 좌표계와 투영법을 통해 결정된 직교 좌표 (미터 단위)

## shp to osm conversion using JOSM

- 국토교통부 정밀도로지도에서 제공되는 A1_NODE.shp도 JOSM 플러그인을 통해 josm gui에서 열어서 편집할 수 있네요

## drawing map!

- 대학 캠퍼스에 RTK-GNSS + 노트북 + 킥보드 타고 돌아다니면서 GPS 수집 -> rosbag 기록 -> osm 데이터로 변환
- DGIST ARTIV 팀은 어짜피 카메라, cv 기술 빵빵하니 정밀도로지도의 차선을 맹신하고 이용하기보다는, 주행유도선 제작에 참조용으로 사용했다고 합니다.

## mapping with SLAM

- 정밀도로지도 자체만으로 굴리기보다는, 주행유도선 참조용으로 쓰고
- 실제 차량이 세상을 인식하는 방식은 라이다 PCD, 측위에 더 가까운 형태로 보이는 것 같습니다.
- (코멘트) 이 연산 많은 pcd 관련 부분을 저희 시뮬레이터에서 ground truth로 제공하고 처리하는 시늉이라도 필요하지 않을까 싶습니다.

## NMEA

- The national marine electronics association (NMEA)
- nmea_navsat_driver
- ros_nmea_driver
- ros_driver 찾아보기

### artiv_nmea_driver가 발행하는 토픽

- 총 7개. 이중 4개는 nmea_navsat_driver의 것과 동일.
- gps_fix (sensor_msgs/NavSatFix)
    - `/fix`와 동일
    - GNSS 센서가 측정한 GPS 좌표
    - 항상 발행
- gps_vel (geometry_msgs/TwistStamped)
    - `/vel`과 동일
    - 단순 pos_fix 외에도 다른 정보를 바탕으로 속도 계산해 발행할 것.
    - 조건부 발행
- gps_deg (std_msgs/Float64)
    - NMEA 데잍터를 통해 차량의 heading angle을 파싱할 수 있다.
    - 북=0도, 동=90도
- gps_yaw (std_msgs/Float64)
    - gps_deg의 회전 판.
    - -180 ~ 180까지.
    - 0=east, 90=north, -90=south
- utm_fix (geometry_msgs/PoseStamped) 
    - gps 발행 좌표계는 위도/경도 단위. 이걸 UTM 좌표계로 변환하자 (미터 단위)
- time_reference (sensor_msgs/TimeReference)
    - 항상 센서들은 자신이 그걸 측정한 순간 시각 정보를 포함해서 넘겨줘야 함.
    - 그리고 그걸 처리하는 미래 소프트웨어가 그 정보를 가지고 적절히 처리할 것

### error types

- type 1 Fatal : invaild Checksum, Device Connection Fail
- type 2 Error : Value Error, HDOP exceeds 3, RTK is not Fixed

- (코멘트) 생각보다 저희가 새로 추가하게 될 토픽이 적을 수도 있다는 생각입니다. common_interfaces나 tf에서 제공되는 메시지 타입들이 충분히 실제 차량 구동에 있어 쓰일 만한 부분들일테니 충분히 공부해놓고 숙지하는 게 좋아 보입니다.


## pyroutelib3

- openstreetmap 데이터를 이용해 경로 생성 가능 (온라인/로컬 둘 다 가능)

```python
from pyroutelib3 import Router # Import the router
router = Router("<transport mode>", "<path-to-.osm-file>")

start = router.findNode(lat, lon) # Find start and end nodes
end = router.findNode(lat, lon)

status, route = router.doRoute(start, end) # Find the route - a list of OSM nodes

if status == 'success':
    routeLatLons = list(map(router.nodeLatLon, route)) # Get actual route coordinates
```

- `"<transport mode>"`는 `"car"`로 설정했다.
- 한계
    - 역주행 금지하지 않음
    - way로 node 간 연결이 되어있지 않은 경우, 차선변경을 하지 못한다.
- 그리하여 이 경로생성을 통해 최종 주행유도선을 구하는 게 본 목적이다.

## OSM Formatter

- ARTIV 연구팀이 사용하고 있는 HD map information publisher의 핵심 : ARTIV_SHUTTLE_NAVI
    - 주행유도선을 구성하는 node에 도로 시설물 정보를 사전에 담아놓은 뒤, node 데이터를 제공하는 형태이다.
    - 그러나 국토교통주 정밀도로지도에서 A1_NODE의 간격은 균일하지 않다. (애초에 고가도고, 교차로 등 의미가 있는 곳에만 촘촘하게 붙여놓았고, 교통표지가 있더라도 굳이 붙이진 않은 걸로 보임)
    - 그렇다면 사전에 A1_NODE의 간격을 촘촘하게 보간해보자..!
    - haversine 공식.
    - 고려해야 할 점 : node ID 부여 방식.
- NODE 간격 세분화 및 균일화 - 달성

### 차로 변경 way 형성 기능

- B2_SURFACELINKMARK 파일로 차선정보 긁어오기. 
- https://dgist-artiv.github.io/hdmap/2021/02/09/artiv-osm-formatter-part2.html
- 노드 기준으로 가상의 way들을 거미줄처럼 엮어주는 것으로 보임. 마치 지역경로계획 graph planning에서 precomputating 하는 것 같은 것으로 보입니다. 이 때 차량 역학 관점으로 가능한 것만 살려놓고, 또 weight를 부여하는 방식도 생각해봐요

## global planning

# HW/Communication

- CAN에 대해 알아보자 (controller area network)
- 1983년 독일 보쉬사에서 개발해 현재 대부분 자동차에서 사용되는 표준 통신 규격.

# Control/Planning

## local path planner

이를 위한 model predictive trajectory generator(MPTG)

- 차량의 상태, 물리를 나름 반영하고 predictive하게 궤적을 생성하는 걸로 보임.
- A*, dijkstra 등 path planning algorithm 중 하나.
- state lattic 활용?

## lane keeping assist

## cruise control based on PID

## vision and GPS data integration driving

- 차량이 도로를 주행하려면 어떤 데이터를 이용해야할까?
- 여러 주행 방법 중 computer vision을 이용한 차선 주행 채택 (ARTIV)
- 사실 실제 차량이 정밀도로지도를 맹신하지 않고 카메라와 라이다로 열심히 측위하는 것은.. 결국 차량이 자기 자신의 좌표를 100% 확신할 수는 없기 때문이다.
- 시뮬레이터의 경우 그걸 ground truth로 만들어버리면 당연히 정밀도로지도 가지고 주행하는게 사기이다.


> HD-Map측에서 주는 30개의 점들이 갱신되는 속도가 약 10fps로 매우 느리다는 것이다.
> 반면 Vision측에서 주는 경로 위의 점들은 빠를 땐 무려 60fps, 아무리 느려도 10fps 이상이다.

- 이걸 보면, 일종의 tradeoff가 있는 걸로 보임

# integrated SW