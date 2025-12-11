# start

```bash
# download git repositories!
./first_git_clone.sh

# install dependencies! (including apt)
./init.sh

# go quick_ws and start!
cd quick_ws
. run.sh
```

## autoware에 대하여

ROS 기반의 오픈소스 자율주행 소프트웨어 스택입니다. 일본의 나고야 대학, tier IV 등과 관련 되어있습니다.

### Ad API

Ad API라는 노트북, 스마트폰 기기, 조이스틱 등 다양한 기기와 상호작용할 수 있는 표준화된 인터페이스를 제공합니다.

### 컴포넌트 기반 설계

`autoware_core`, `autoware_universe` 등의 레포에서 인지, 플래닝, 컨트롤 등 컴포넌트 단위와, 그 하위 패키지들을 제공해서 부분적으로 개발하고, 교체할 수 있습니다.

### 벤더별 확장

다양한 sensor, vehicle에 적용될 수 있도록 명세와 도구를 제공합니다. Tier IV, AWF 등 다양한 벤더별 솔루션을 제공합니다.

### 데이터

rosbag, 머신러닝, 강화학습 등을 지원하기 위한 도구, 데이터들도 생겨나고 있습니다.

## dependencies (repo)

- autoware_core
- autoware_universe
- autoware_lanelet2_extension
- autoware_launch
- autoware_msgs
- sensor_component_description
- tier4_autoware_msgs

### autoware_core

high-quality, stable한 autoware 관련 ROS 패키지들을 저장하는 레포입니다. 정말 핵심 기능만 존재하며, 실질적으로 autoware를 돌릴려면 `autoware_universe`가 필요합니다.

### autoware_universe

개발, 실험 등 목적의 ROS 패키지들이 위치해 있습니다. 실용적인 패키지들도 많이 있습니다.

```
Autoware.ai (2015-2020) → Autoware.auto (2020-2021) → Autoware.universe (2021-현재)
```

Autoware는 이와 같은 형태로 버전업이 진행되어 왔습니다.

### autoware_launch

autoware 스택 및 시뮬레이터 스택의 메인 엔트리포인트인 `autoware_launch`가 들어 있습니다. 이외에도 `sample_vehicle`, `sample_sensor_kit` 등이 있어서 다른 컴포넌트 실험 용으로 사용할 수도 있고, 이와 같은 vehicle과 sensor 명세를 맞추어 실차에 맞는 모델을 개발할 수 있습니다.

### autoware_msgs

autoware 패키지들을 위한 인터페이스들을 모아놓은 레포입니다.

### tier4_autoware_msgs

tier4에서 autoware_msgs에 대해 확장이 필요한 인터페이스들을 모아놓은 레포입니다.

tier4는 autoware foundation와 긴밀한 협업 관계에 있는 자율주행 관련 회사로, 소프트웨어 외에도 센서 하드웨어를 제공하기도 합니다.

### autoware_lanelet2_extension

`autoware_lanelet2_extension` 단일 패키지를 담고 있는 작은 레포입니다.

### sensor_component_description

앞서 autoware_launch에서 말한, sensor_kit에서 구체적인 센서들이 가질 수 있는 명세입니다. camera, imu, livox, pandar, radar, velodyne, vls 등의 센서에 대한 명세를 제공합니다.

명세는 주로 urdf (base_link 같은 tf 및 프레임들을 묘사하는 xml 파일)와 meshes (시각화)로 구성됩니다.

autoware는 해당 명세의 숫자를 직접 채우지 않고도, 마치 SLAM으로 지도를 만들듯이, 차량을 실제 주행하고 데이터에 기반해서 센서의 urdf를 작성하고 보정해주는 calibration 관련 도구들을 많이 지원해줍니다.

## dependencies (apt)

autoware 패키지 중, 의존성이 너무 깊어지거나, 굳이 코드로 수정할 것 없는 패키지들은 `sudo apt install ros-humble-*`으로 apt 상으로 다운받아 설치했습니다.

apt로 받은 패키지 중 일부는 버전 충돌이 일어날 수 있어서, 다운받는 식으로 변환하였습니다.

apt로 받은 패키지 목록에 대한 설명은, quick_ws/README.md에서 통합해서 진행합니다.

## sample-map-planning

이 데이터는 일본 가시와시 지바현에 위치한 도쿄대학 가시와 캠퍼스 주위의 HDMap (정밀도로지도, 벡터지도) 및 pointcloud 데이터가 담긴, autoware에서 샘플로 제공하는 데이터입니다.

### lanelet2

독일의 FZI, Research Center for Information Technology 연구기관에서 개발한, cpp 기반의 고정밀 지도 데이터 형식 및 라이브러리입니다.

사용되는 파일 형식은 .osm, 그 중 .josm의 확장격 성격을 띠었으며, node와 way, relation으로 구성됩니다.

lanelet2 데이터 형식에 대한 io, 바이너리 인코딩, 쿼리 등을 지원합니다.

autoware에서 사용하는 HDMap의 형식은 lanelet2를 따릅니다. LaneletMapBin이라는 형태로 통신을 하되, lanelet2 라이브러리가 제공하는 쿼리, 함수 등을 이용해 효율적으로 벡터정보를 조회해 플래닝 등 컴포넌트에서 사용할 수 있습니다.

