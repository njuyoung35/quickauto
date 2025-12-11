# mapp

## rviz

- OpenSceneGraph? 객체 계층 구조 생성
- tf2 좌표계 변환
- opengl을 통한 실제 그래픽 출력
- qt 위젯에 통합된 렌더링 결과
- ogre 3d rendering engine
- 위도우 시스템 : x11 / wayland

## osm

```bash
sudo apt install ros-humble-lanelet2
```

### 지도 타일 관련 조회

- leaflet
- openlayers

- nominatim : 논리적 핀포인트 찍어줌 (파란색)
  - 건물 중심, 가상점, 지역구, 도로 중심 뭐 이런거.. (반드시 도로는 아님)

## overpass

## 지리적 물리량

- 위도/경도/고도 WGS84 : 글로벌 기준, 맵 매칭, V2X 통신
- UTM 좌표계 : 지역적 직교좌표로 변환 가능
- 로컬 cartesian 좌표계
    - ego-centric
    - map-centric
- 고정밀 맵 앵커 포인트 기반 상대좌표

```py
class LocalizationSystem:
    global_coord = (lat, lon, alt) # WGS84
    local_origin = (lat0, lon0) # 로컬 좌표계 원점
    local_coord = (x, y, z) # 미터 단위
```

통신 프로토콜

```
{
  "position": {
    "wgs84": {"lat": 37.123, "lon": 127.123, "alt": 45.2},
    "local": {"x": 1234.5, "y": 567.8, "z": 2.1},
    "coordinate_system": "UTM-52N",
    "timestamp": 1234567890
  }
}
```

### 정밀도 요구사항

- 차선 수준 정밀도: 10-20cm 오차 허용
- 실시간 성능: 100Hz 이상 위치 업데이트
- 일관성: 좌표계 간 변환 시 시간동기화

## 종합 아키텍처

- 고빈도 실시간 데이터 레이어 (RViz 기반)

- 지도 레이어: 웹 기반 하이브리드 솔루션

- webview + leaflet/maplibre 되냐?