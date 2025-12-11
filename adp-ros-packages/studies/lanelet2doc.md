# lanelet2

## primitives

- Point
- LineString = Point[]
- Polygon
- Lanelet
- Area
- RegularElement

unique id + attr(k, v)

lanelet attr에 대한 접근은 내부적으로 더 효율적임.

### 태그

- type
- subtype
- no_issue : lanelet2_valdiation의 검사 결과

### 좌표계

- 2d 평면.
- 필요시 높이 WGS84 추가.
- 높이가 실제 미터가 아닌, 저상도로, 고가도로를 구분하는 카테고리 숫자로 쓰일 수도 있음 (2.5d)

### LineString

- 유효하려면, 꼬이면 안됨.
- 타입 왠만해서 포함
- 차선 경계로서
- 심볼
- 트래픽 사인
- 신호등

### Polygon

주로 linestrinf 쓴다.

그래도 area나 region of interest 표현하는데 쓸 순 있음.

### Lanelet

좌우 linestring 하나씩 한쌍

ref elem 참조 가능
speed limit은 반드시 lanelet별로 제공되어져야함
lanelet 이용 가능 참여자 명세해야함
ㄴ 이 세 가지 중요

subtype
location (urban, nonurban, non)
이 둘을 조합해봐요
ㄴ 이걸로 참여자, 속도제한 유추 힌트
ㄴ 이 암시적 추론은 오버라이딩될 수 있음

추가적으로, 도로이름, 표면성분, regionISO 코드 명세 가능

차선 변경 결정은 lanelet이 아닌, border에 영향
방향관련 태그. 참여자와 조합

area=multipolygon
주차, 자유공간, 식생, 진입금지, 빌딩, 교통섬 등 표현 가능
area는 방향성 없는 ls 비슷
ls로 cw ccw 방향으로 둘러짐.
정확히 하나 이상 ls 공유해야 인접한 area로 판단
이 공유 바운더리가 from to 제한 속성을 가질수있음

hole

### lanelet& area tagging



### regular elements

ls나 area가 참조

TrafficLight, TrafficSign, SpeedLimit and RightOfWay

커스텀 추가도 가능

· 관계(Relation) 형태: 여러 맵 요소들을 연결하여 규칙 정의
· 역할(Role) 기반: 각 참조 요소가 규칙 내에서 하는 역할 정의
· 동적 제어 가능: 시간/조건에 따라 변하는 규칙 표현 가능

#### parameter

## 아키텍처

- 데이터 공유 (포인터, 아이디)
- 