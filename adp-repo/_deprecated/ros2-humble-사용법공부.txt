로봇 응용(앱) = 하나의 목적을 위해서 모션을 생성하는 프로그램들의 집합
목적을 위해, 센서로 주변 감지, 모션 계획, 액츄에이터 활용해 집합적 모션 만들어냄
여기서 ros, os는 커널은 아니고, 기존 커널 위에서 실행되는 실행파일+라이브러리의 집합일 뿐 (middleware)
	메타 운영체제라고 부르기도.
	이 책에서는 ros2 다룸.
	
{ Ros Node
{ Ros Client libraries
{ RCL/RMW

    ----DDS----
	
networked computers

# B.2 구조와 특징

노드 = 실행 단위
1 실행파일 : n 노드에 대응될 수 있음. 그러나 1:1 가정.
1 노드 : 1 .elf / .so = 하나의 프로세스
	그러나 so파일로 만들어진 노드는 component container라고 부르는 프로세스 안에서 다른 노드들과 공존 가능
	ㄴ (ros에서) component container = so 라이브러리들을 동적으로 적재할 수 있는 프로세스

- 발행-구독(publish-subscribe) 모델
	토픽=노드들 사이 통신 링크 : 단방향 통신 링크
	서버 역할 하는 노드 = 서비스 서버
	서비스 서버에 요청하고 응답 기다리는 노드 = 서비스 클라이언트
	고로, 노드 하나는 발행자, 구독자, 서비스서버, 서비스클라이언트 자격의 일부 또는 모두를 가질 수 있음
- 다양한 운영체제 지원 : 심지어 컴퓨팅 파워가 작은 micro-controller 기반 임베디드에서도 가능. (micro-ROS)
	RTOS(real-time operating system)에서, 만약 bare metal이더라도 micro-ROS 가능
- 다양한 프로그래밍 언어 지원 : 토픽 기반 메시지 통신의 인코딩은 언어독립적.
	RCL 라이브러리로 제공
- DDS(data distribution service) : OMG(object management group)가 정의한 DDS 통신 middleware 표준 사용
	DDS는 통신 개체들 사이에 발행-구독 모델의 메시지 통신 지원.
- 분산 컴퓨팅 지원
- 강화된 실시간성 지원
	로봇응용은 물리세계에서 모션을 생성하기 때문에, 제한된 시간 안에 정해진 동작을 수행해야 한다.
	센서입력 - 모터출력까지의 종단간 지연 시간이 < 시스템 설계자가 정한 마감시한보다 작아야.
	각 노드가 수행하는 작업, task는 예측 가능한 응답 시간을 가져야 한다.
	ROS는 각 태스크의 응답시간 개선을 위해
		메모리 로킹(memory locking) : 실행코드와 데이터를 RAM에 상주시킴
		실시간 우선순위 스케줄링(real-time priority scheduling) 활용 가능
- 강화된 보안 지원
	dds의 보안 명세를 활용하여 강화된 보안 기능 제공
	node discovery : 노드들이 통신을 위해 상호 식별 수행. PKI(public key infrastructure) 이용
		주고 받는 메시지 암호화, 노드들에 대한 접근 제어 지원

setup.bash : 작업공간 정의. (ROS 개발 수행하고 있는 디렉토리)
/opt/ros/humble : 핵심 작업공간 = underlay
사용자가 만든 ~/f1tenth_ws : overlay
source /opt/ros/humble/setup.bash
	현재 쉘에서 ros패키지 사용 가능.
	내 overlay에서 패키지빌드하여 /install 경로에 setup.bash와 local_setup.bash 생성.
	이거 역시 source시 해당 패키지들 사용 가능.
	
노드의 통신법 (각각 매체라고 부름)
	토픽 : 단방향 통신 링크
	서비스 : 단발성 요청-응답(request-replay) 모델 지원하는 양방향 통신 링크
	액션 : 토픽들과 서비스들을 기반으로 만들어진, 장시간 수행되는 서비스. 단발성으로 요청하지만 응답은 주기적으로 반복됨
	파라미터 : 노드 외부에서 그 값을 설정할 수 있는 노드 내부의 변수이다.
		각 노드는 자신의 파라미터들을 다른 노드들이 조회하거나 설정할 수 있도록 관련 '서비스' 제공.
노드를 개발하고 ros패키지로 만들어야, 배포할 수 있다. 하나의 패키지 = 여러 노드 or 여러 실행 파일

ros2 : /opt/ros/humble/bin에 있는 python3 스크립트 파일임.
	#!/usr/bin/python3
	서브명령 : run, ..
	ros2 run <패키지 이름> <실행파일>
	ros2 launch <패키지 이름> <launch filename>
		multisim.launch.py에서 여러 개 노드 실행.
		py 외에도 xml, yam 등 형식으로 제공 가능

ros2 node list
ros2 node info /turtlesim
Subscribers:
	/parameter_events: rcl_interfaces/msg/ParameterEvent
	..
Publishers:
	/parameter_events: rcl_interfaces/msg/ParameterEvent
	..
Service Servers:
	..
Service Clients:
	..
Action Servers:
	..
Action Clients:
	..
	
# B.3.3 ros 토픽
	구독자 노드는 발행자 노드들이 어느 컴퓨터에서 존재하는지, 어떤 UDP 포트를 사용하는지 몰라도 됨
	발생자 또한 구독자에 대해 동일함.
	컴퓨터 IP 주소와 UDP 포트 같은 저수준 통신 - DDS middleware가 관리

	topic은 DAGs를 구성함.
	
$ rqt_graph
	동그라미 : 노드
	네모 : 통신 매체
	화살표 : 메시지 흐름 방향
	
$ ros2 topic list
/parameter_event
/rosout
..

$ ros2 topic list -t
	메시지 타입의 이름 함께 출력
	
$ ros2 interface show <메시지 타입>
	메시지 타입 예를 들면 [geometry_msgs/msg/Twist]
	
$ ros2 topic echo <토픽 이름>
$ ros2 topic info <토픽 이름>
$ ros2 topic pub -once <토픽 이름> <메시지 타입> <내용>
	"linear: x: 2.0, y: 0.0, z: 0.0, angular: x: 0.0, y: 0.0, z: 1.8" (yaml 형식)
$ ros2 topic pub -rate <hz> <토픽 이름> <메시지 타입> <내용>
	<hz> hz 주기로 계속 발행.
$ ros2 topic hz <토픽 이름>
	토픽으로 지나가는 메시지들의 빈도를 hz 단위로  출력하라.
	그 출력에서 나오는 average rate가 62hz이면 1초에 62개 메시지 지나간다는 뜻
	
# B.3.4. ROS 파라미터

integer, float, boolean, string, list 같은 타입을 가질 수 있음

$ ros2 param list
$ ros2 param describe <노드 이름> <파라미터 이름>
Parameter name: ..
	Type: ..
	Description: ..
	Constraints:
		Min value
		Max value
		Step
$ ros2 param get <노드 이름> <파라미터 이름>
$ ros2 param set <노드 이름> <파라미터 이름> <값>
$ ros2 param dump <노드 이름>
	get으로 하나하나 구하는게 아니라, 그냥 그 노드 내 있는 파라미터 전부 찍어 보여줌 (yaml 형식)
$ ros2 param load <노드 이름> <파일 이름>
	파일로는 yaml 파일이면 될 듯
	read-only 파라미터는 덮어씌워지지 않는다.
	$ ros2 run <노드 이름> <패키지 이름> --ros-args --params-file <파일 이름>
	으로 실행했다면, 초기 파라미터 값을 넣어버릴 수 있다.
	
# B.3.5. 토픽 저장과 재생

$ ros2 bag record -o <rosbagname> <topic>

잃게 저장된 rosbag 파일은 추후 turtle_teleop_key 같은 컨트롤러 노드를 실행하지 않고 과거 메시지 시퀀스를 재현할 수 있다.
$ ros2 bag play <osbagname>
	그럼 얘가 turtle1/cmd_vel 토픽을 발행한다! 동일한 타이밍으로!
