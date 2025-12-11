#set heading(numbering: "A.1.")

= 목차

- Dockerfile 분석 (/opt/ros/humble)
- roscpp 패키징, 개발도구 분석 (ch_roscpp_basic)
- ros 개발도구 분석 (ch_ros_usage)
- 스택 분석 (f1tenth_ws, forza_ws)
- rcl 분석
- rviz2, f1tenth_gym, fox 시뮬레이터 분석

= Dockerfile_misys_forza

+ FROM nvidia/cuda:12.4.1-devel-ubuntu22.04
  - /opt/nvidia (1.4GB) 설치
  - /usr/local/cuda-12.4 (4.7GB) 설치
+ gcc, g++ 설치
+ timezone, locale(언어) 설정
+ ca-certificates curl gnupg software-properties-common 설치
+ ros/rosdistro로부터 Ubuntu 버전 가져옴
+ ros-humble-desktop 설치
  - /opt/ros/humble (343MB) 생성
  - gui : rviz2, rqt, rqt_graph, rqt_console
  - 시뮬레이션 : gazebo
  // - 기본 라이브러리 : navigation2?, movelt2?
+ ros-dev-tools 설치
  - colcon-common-extensions 설치 (colcon, ament_cmake, catkin 통합)
  - vsctool, rosdep 설치
  // - ros2cli?, ament_cmake?, 메시지/서비스 생성 도구
  - ros-build-essential 설치 (cmake, git, python3)
+ ros-humble-laser-proc (urg_node2 의존성) 설치
+ `$ rosdep init`
+ 유저 ID, 비번 생성
+ /home/misys 패키지들 설치

리눅스 운영체제를 가상화한 바이너리와 라이브러리는 필수적이고 가벼운건 /bin, /lib, /lib64, 추가적인건 /usr/bin, /usr/lib, /usr/lib64 등에 있다고 보면 된다.

= roscpp 패키징, 개발도구 분석 (ch_roscpp_basic)

== c++ 패키지 생성: package.xml과 CMakeLists.txt

```sh
$ # underlay 후 ros2_ws/src 생성후 그 위치에서 
$ ros2 pkg create --build-type ament_cmake --node-name minimal_fusion_node minimal_fusion_example
$ cd ~/ros2_ws
$ colcon build --packages-select minimal_fusion_example
$ # 이제 install/setup.bash를 overlay로 실행 가능
```

ament_cmake는 .srv, .msg, .action 등의 선언적 파일을 c 계열의 언어 텍스트로 변환해주는 도구이다.

```sh colcon build --packages-up-to <패키지>``` 옵션을 사용하면 해당 패키지의 의존성(package.xml)을 찾아 모두 빌드해준다.

=== package.xml에 대하여

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>minimal_fusion_example</name>
  ...
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
```

- \<depend\> : 타겟 패키지 빌드, 실행 시 의존성.
- \<build_depend\> : 타겟 패키지를 빌드할 때 필요한 의존성
- \<build_export_depend\> : 타겟 패키지가 공개하는 헤더파일들이 포함하는 다른 헤더파일들이 속한 패키지들 명세.
- \<exec_depend\> : 타겟 패키지를 실행할 때 필요한 so 라이브러리들, 실행 파일들, 파이썬 모듈들, 런치 스크립트들, 기타 파일들 명세

ROS 패키지가 의존하는 다른 패키지는 반드시 ROS 패키지일 필요는 없다.
- ROS 패키지 : https://github.com/ros/rosdistro 에서 제공하는 <distro>/distribution.yaml 조회
- 비ROS 패키지 : https://github.com/ros/rosdistro 에서 rosdep/base.yaml (apt가 관리하는 시스템 패키지 목록), rosdep/python.yaml (파이썬 패키지 목록) 조회

=== CMakeLists.txt에 대하여

CMake : cross-platform make

=== 소스코드

src/minimal_fusion_example/src/minimal_fusion_node.cpp

```cpp printf("Hello World!");```를 수행하는 간단한 파일이다.

== C++ 패키지 작성: 노드, 토픽, 파라미터

src/minimal_fusion_example에서 ./src/minimal_publisher_node.cpp와 ./src/main.launch.py를 작성한다.

본격적인 roscpp 패키지를 개발하기 위해 package.xml에 ```xml <depend>rclcpp</depend>```와 ```xml <depend>std_msgs</depend>```를 추가한다.

main.launch.py를 작성하자.

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='minimal_fusion_example',
            executable='minimal_publisher_node',
            name='talker_w_period_1s',
            output='screen',
            emulate_tty=True,
            parameters=[ {"timer_period": 1000} ],
            remappings=[ ('topic', 'topic_1') ]
        ),
        Node(
            package='minimal_fusion_example',
            executable='minimal_fusion_node',
            name='talker_w_period_1s',
            output='screen',
            emulate_tty=True,
            parameters=[ {"timer_period": 1000} ]
        )
    ])
```

- remappings : 해당 노드가 발행하는 토픽의 이름을 바꿈.
- output : 노드가 생성하는 로그 문자열을 터미널 스크린에 출력할건지
- time_period : \*0.001Hz의 주기를 가짐.

이제 minimal_publisher_node.cpp 파일을 작성하자.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node {
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

사실 rclcpp를 사용하는 스타일에도 역사가 있어왔다.

- ros1 시절에는 old_school (not_composoable) 방식을
- ros2 시절 초창기에는 local_function(lambda) 방식을
- ros2 현재에는 member_function(클래스)방식을 사용한다.

https://github.com/zhangrelay/ros2_tutorials/tree/master 을 돌아다니다보면 해당 이름으로 구분되어 있는 소스코드들을 확인할 수 있다.

=== 실행

```sh
$ source /opt/ros/humble/setup.bash
$ cd ~/ros2_ws
$ colcon build --packages-select minimal_fusion_example
$ source install/setup.bash
$ ros2 launch minimal_fusion_example main.launch.py

$ ros2 node list
$ ros2 topic list
$ ros2 topic hz /topic_1
```

=== 메시지 타입

std_msgs
geometry_msgs

== TF2

TF2 : 좌표계들 사이의 변환을 시시각각 관리

= ros 개발도구 분석

- ros2
- colcon
- ament_cmake
- rviz2

= ros2 documentation

== Concepts

=== Basic Concepts

ROS 2는 강타입, 익명 pub/sub 메커니즘 기반의 미들웨어입니다. ROS 2 시스템의 심장은 ROS graph입니다.

==== Nodes

노드는 ROS 2 graph의 참여자입니다. 노드는 client library를 사용해 다른 노드와 통신합니다. 노드는 같은 프로세스 내, 또는 다른 프로세스끼리, 또는 다른 머신과도 통신할 수 있습니다. 일반적으로 노드는 ROS graph에서 계산의 단위이며, 각 노드는 하나의 논리적 작업을 해야 합니다.

노드는 pub/sub, service client/server, action client/server로서 역할 할 수 있습니다. 노드는 configurable parameters를 제공해 런타임에 behavior를 바꿀 수 있습니다.

노드 사이의 연결은 분산 discovery 프로세스에 의해 설립됩니다.

==== Discovery

노드에 대한 디스커버리는 아래 깔린 ROS 2 미들웨어에서 자동적으로 수행됩니다. 다음으로 요약 가능:
+ 노드가 시작되면 같은 ROS_DOMAIN_ID 네트워크에 있는 다른 노드에게 자신의 존재를 통지한다. 다른 노드들은 그 통지에 대해 각자 자신의 정보와 관련해 응답해서 그 경우 조합에 따라 적합한 통신이 이루어진다.
+ 노드들은 주기적으로 그들의 존재를 통지한다.
+ 노드들은 종료될 때도 다른 노드들에게 통지한다.

노드들은 그들이 호환되는 QoS 설정을 가질 때에만 커넥션을 수립할 것이다.

디스커버리는 사실상 미들웨어 수준에서 제공되는거라, 로직 상 크게 만지는 건 없는 걸로 보임.

==== Interfaces

ROS 앱들은 다음 세 가지 유형 중 하나의 인터페이스로 소통한다: topics, services, actions. ROS 2는 간단한 설명언어를 쓴다. IDL(interface description language).

===== Messages

응답을 딱히 기대하지 않고, 단순히 보내는 데이터 유형.

`msg/` 경로에 `.msg` 파일들 작성.

```msg
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
```

필드 타입은 built-in이거나, 사용자정의 타입일 수 있다.

배열에 관한 타입:
- static array : `std::array<T, N>`
- unbounded dynamic array : `std::vector`
- bounded dynamic array : `custom_class<T, N>`
- bounded string : `std::string`

bounded의 경우 `string<=10[<=5]` 이런식으로 범위 표현할 수 있음

```msg
fieldtype fieldname fielddefaultvalue
```

기본값까지 할당할 수 있는데, 문자열 배열이나, 복잡한 타입에 대해서 기본값 설정하는건 아직 지원이 되지 않는다.

```msg
constanttype CONSTANTNAME=constantvalue
```

파라미터, yaml 파일이나 구성을 하도 바꾸고 다니니, 절대 고정시킬 값은 등호를 써서 상수로 표시해주면 된다.

===== Services

```srv
string str
---
string str
```

서비스는 request/response가 있는 통신을 말하므로, `---` 줄로 요청, 응답을 나눠 데이터들을 적어주면 된다.

===== Actions

```action
<request_type> <request_fieldname>
---
<response_type> <response_fieldname>
---
<feedback_type> <feedback_fieldname>
```

액션은 request/response/feedback 모두 있는 통신을 말한다. 서비스의 경우 클라이언트는 서버를 기다렸지만, 액션의 경우 기다리지 않는다. long-term 통신은 액션을 사용한다.

액션에서 응답이 늦게 올 수 있으므로, 클라이언트는 자기 할 일을 하면서도, 중간중간 서버로부터 피드백을 전달받는다. 그리고 서비스 유형과 다르게, 액션의 경우 인터럽트가 발생하여 요청과 작업이 전부 중단될 수 있다.

==== Topics

===== 발행자/구독자

발행자와 구독자는 토픽 개념을 통해 서로 연락할 수 있다. 토픽은 개체들이 서로를 발견할 수 있도록 해주는 이름이다. 발행자와 구독자 모두 토픽에 접근하기 위해 문자열을 통해 이름으로 접근한다.

특정 토픽에 대하여서는 발행자 0개 이상, 구독자 0개 이상이 존재할 수 있다.

발행자와 구독자는 필요에 따라 오고 갈 수 있다, 즉 디버깅과 introspection은 자연스러운 확장일 뿐이라는 것이다. ros2 bag record도 단순한 구독자 중 하나일 뿐이다.

===== 익명성

일반적으로 구독자는 데이터를 얻을 때, 어느 발행자가 보내는 것인지 신경쓰지 않거나 모른다. (물론 원한다면 알아낼 수는 있습니다) 이 아키텍처의 장점은 시스템의 나머지 부분에 영향을 미치지 않고서 발행자와 구독자를 원하는 대로 교체할 수 있다는 것입니다.

create_publisher 같은 함수는 있는데, 교체, 수정 이런건 아직 보지 못했는데..?

===== 강타입

```msgs
uint32 field1
string field2
```

이러한 형태의 ROS 메시지는 각 이름의 개체가 해당 타입을 가질 것을 보장한다.

각 필드에 대한 의미론은 잘-정의되어 있다. 예를 들어 IMU 메시지의 경우 측정된 각속도 3차원 벡터를 포함하고, 각 차원은 radians/second 단위로 명세된다. 이 외의 해석은 허용되지 않는다. ROS 타입들은 하나의 의미론, 해석에 대응된다고 보면 된다.

==== Services

ROS 2에서 서비스는, remote procedure call을 지칭한다. 즉, 노드는 다른 노드가 계산을 하고 결과를 반환할 것을 원격으로 요청할 수 있다는 것이다 (원격 프로시저 콜)

ROS 2에서, 서비스는 가능한 빨리 반환될 것으로 기대된다. 그리하여 클라이언트는 결과를 기다린다. 그러므로 서비스는 long-time 프로세스에 대해 사용되어지면 안된다. 그러므로 이는 다른 것에 선점되거나 예외 상황에 처할 가능성이 있는 프로세스에는 적합하지 않다. 만약 그러한 long-time 프로세스 계산을 처리하고자 한다면, action을 고려하라.

===== Service server

서비스 서버는 요청을 수락해 계산을 취하고 결과를 반환하는 개체이다. 하나의 서비스 이름에 대하여서는 반드시 하나의 서비스 서버만 존재해야 한다. 하나의 서비스 이름에 대해 여러 서비스 서버가 있다면, 무엇이 선택되어질지에 대해서 undefined이다.

===== Service client

서비스 서버에게 요청을 보낸다. 그리고 결과가 올 때까지 기다린다.

서비스 서버와 다르게, 클라이언트는 여럿 존재할 수 있다.

==== Actions

취소되거나 목표달성에 의해 선점될 수 있는 피드백과 능력을 갖춘 long-running remote procedure call을 지칭한다. 예를들어 로봇의 고수준 상태머신은 navigation subsystem에게 waypoint를 travel하라고 말할 수 있다, 이것은 초\~분 단위를 요구하는 작업 단위이다. 그리고 navigation subsystem은 action에게 피드백을 제공할 것이다.

액션은 긴 프로시저에 걸쳐 실행될 것으로 기대하며, 연결을 유지하고 모니터링하는데 약간의 오버헤드는 있을 것이다. 즉각적인 원격 프로시저 콜이 필요하다면 service를 고려하라.

===== Action server

요청을 수락하고 어떤 절차를 수행한다. 또한 액션 서버는 피드백을 보낼 의무가 있으며, 취소/선점 요청에도 반응해야 한다.

요청 / 응답 / 피드백

서비스 서버와 비슷하게, 하나의 액션 이름에 대해서는 하나의 액션 서버만 있어야 한다.

===== Action client

초기 메시지를 만들어 액션 서버에게 전송하고 액션 서버의 응답을 기다린다. (그 과정에서 서버에게 피드백을 제공받음)

==== Parameters

ROS 2에서 파라미터는 개별 노드들과 관련된다. 파라미터는 노드의 startup(그리고 런타임)을 구성하는데 쓰인다. 파라미터의 생존주기는 노드의 생존주기에 묶여 있다.

파라미터는 노드명, 노드 네임스페이스, 파라미터명, 파라미터 네임스페이스를 통해 addressing된다. 파라미터 네임스페이스 제공은 옵셔널이다.

각 파라미터는 key, value, descriptor로 구성된다.

키는 다음 타입들 중 하나이다 : bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[]

기본적으로 descriptor는 비어있지만, decriptions, value ranges, type information, additional constraints 등을 포함할 수 있다.

파라미터가 왜 있는가? 토픽, 서비스, 액션은 동적 데이터 교환 (통신 규약)을 말하기 위한 개념이고, 파라미터는 정적 설정 관리 (구성, 설정값, 행동 규칙) 등을 설명하기 위한 개념이다. 컴퓨터 상에서는 단순히 int, float 등의 데이터 구조를 가지더라도 라디안, 속도 등 물리적 단위에 대응되는 의미론을 입혀주고 실제 범위 등 제약을 달아주는 것이 파라미터의 역할이라고 볼 수 있겠다.

===== 파라미터 선언

기본적으로 노드는 생명주기동안 수용할 파라미터들을 선언한다. 즉 startup 때에는 파라미터의 타입과 이름이 잘-정의된다.

어떠한 노드는 모든 파라미터가 초기화되지는 않는다. `allow_undeclared_parameters` 옵션을 참으로 만들면 가능하다.

===== 파라미터 타입

파라미터의 타입은 개요에서 언급한 타입 종류 중 하나에 해당할 수 있다. 일반적으로 런타임 도중 선언된 파라미터의 타입을 바꾸는 것은 실패한다.

만약 파라미터가 여러 타입이 되어야 하고, 코드가 그걸 handle할 수 있다면, `ParameterDescriptor`의 `dynamic_typing`을 참으로 만들어 보아라.

===== 파라미터 콜백

파라미터에게 변화가 생겼을 때 이에 대한 콜백을 노드는 두 가지 형태로 등록할 수 있다.

====== set parameter 콜백

"set parameter" 콜백 : node API의 `add_on_set_parameters_callback`에서 호출될 수 있다, 러스트의 경우 파라미터 핸들을 통해 조작. 이는 불변 `Parameter` 객체들의 리스트를 통과시키고, `rcl_interfaces/msg/SetParametersResult` 를 반환한다.

"set parameter" 콜백은 side-effects가 없어야 한다. 왜냐면 여러개의 "set parameter" 콜백이 연쇄될 수 있으므로.

이 콜백 유형의 주된 목적은 파라미터에게 다가오는 변화를 조사할 능력을 주고, 명시적으로 변화를 거부할 권리를 주는 것이다. https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#a12d535bced9f26b65c0a450e6f40aff8

이 콜백 핸들러는 어떠한 형태의 `set_parameter*`, `declare_parameter*` 메소드 호출에든 호출된다. 상술했듯이 불변 파라미터 객체 리스트를 입력으로 받아, `SetParametersResult`를 반환한다. 이 반환은 파라미터가 should be set or not인지 보여주고, 만약 not이라면 why까지 설명해준다.

```cpp
rcl_interfaces::msg::SetParametersResult
my_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (!some_condition) {
      result.successful = false;
      result.reason = "the reason it could not be allowed";
    }
  }
  return result;
}
```

이러한 콜백 유형이 필요한 이유는, 단순 range, constraints 등 인터페이스 외에, 사용자가 완전한 컨트롤을 지닐 수 있도록 만들기 위한 것으로 보인다. 러스트에서는 api상에서 이런 콜백함수를 넣을 수 없는 것으로 보인다. 심지어 `.constraints`, `.description` 등 메소드는 인자로 그저 문자열을 받는다. 즉, 자연어를 보내고 그걸 해석하도록 만든다는 것이다..

rclrs에서 이 `add_on_set_parameters_callback`은 parameter/service에 private 함수로서는 구현이 되어있다. 이것은 async worker로의 통합을 위해 PR 통과되었으며, 당신은 async worker에 대한 api를 공부해야 할 것이다. 이러한 파리미터 콜백 기능을 구현하기 위해서는

놀랍게도 `declare_parameter<T: ParameterVariant>`에 제너릭이 있다. `trait ParameterVariant`은 `kind()` 메소드를 구현해서 `enum ParameterKind`을 반환하도록 작동해야 한다. 즉, rclcpp의 경우 이러한 커스텀 타입으로의 제약보다는 콜백을 통한 관리를 하는 것이고, 러스트에서는 애초에 제너릭 T에 타입을 명시해 그 동작을 설정할 수 있는 것이다.

그렇게 보면 러스트에서 동작 타이핑이 어떻게 이루어지는건가 싶기도 한데.. 우선 `ParameterVariant` 트레잇은 not dyn compatible이다.

`ParameterVariant` 트레잇은 연관타입으로 `type Range: Into<ParameterRanges> + Default + Clone`을 요구한다. 이는 해당 파라미터의 여러 범위들을 묘사한다. ..정작 그렇게 말하는 것 같지만 구현체를 까보면

```rust
#[derive(Clone, Debug, Default)]
pub struct ParameterRanges {
    float: Option<ParameterRange<f64>>,
    integer: Option<ParameterRange<i64>>,
}
```

이 딴식이다.. 어짜피 range가 의미를 가지는 건 정수 또는 실수일 뿐이라는건가..

두 번째 유형은 "on parameter event" 콜백이다. 이것은 parameter client API의 `on_parameter_event`에서 호출될 수 있다. 이것은 `rcl_interfaces/msg/ParameterEvent` 객체를 통과시키고, 아무것도 반환하지 않는다. 이것은 입력 이벤트에 있는 모든 파라미터가 선언되거나, 변경되거나, 삭제될 때 호출된다. 이 콜백의 주 목적은 성공적으로 허용된 파라미터의 변화에 반응하는 능력을 주기 위함이다.

```rust
let param = node.declare_parameter("int_param")
    .default(10)
    .range(ParameterRange { lower: Some(0), upper: Some(100), step: Some(2) })
    .mandatory() // -> Result<MandatoryParameter, ..>
                 // Mandatory, ReadOnly, Optional 파라미터가 있다.
    .unwrap();
```

러스트의 `get`, `set`, 메소드는 `Parameter` 계열 구조체에 달려있는 메소드이다. 이것은 말 그대로 값을 얻고 설정하는 메소드이다. 즉 콜백 함수를 등록하는 api는 명시적으로 존재하지 않으므로 직접 `std::thread::spawn`을 통해 흐름을 제어해야 할 것으로 보인다.


===== 파라미터와 상호작용

파라미터는 서비스 기반으로 볼 수 있다.

- `/node_name/describe_parameters` : 이름으로 데스크립터 얻기
- `/node_name/get_parameter_types` : 이름으로 타입 얻기
- `/node_name/get_parameters` : 이름으로 값 얻기
- `/node_name/list_parameters` : prefix를 받아 그에 해당하는 파라미터들 반환
- `/node_name/set_parameters` : 파라미터들 개별적으로 set한다. 결과 리스트 형태를 반환한다.
- `/node_name/set_parameters_atomically` : 위의 단순 set과 다르게, 하나라도 set 시도에 실패하면, 모든 파라미터에 대한 set을 포기하고, 실패를 반환한다.

===== 노드 running 시 초기 파라미터 값 설정

노드를 cli상에서 `ros2 run`으로 실행할려고 할 때, cli 인자로 파라미터를 전달하거나, YAML 파일로 전달시킬 수 있다.

```sh
$ ros2 run <package_name> <executable_name> --ros-args -p <param_name>:=<param_value> -p <param_name>:=<param_value> -p <param_name>:=<param_value> ...
```

여러개의 파라미터를 보내려면 매번 `-p` 옵션을 붙여주어야 한다.. 그리고 만약 파라미터 값에 공백이 포함된 문자열이 온다면 bash가 이해할 수 있게 쌍따옴표로 잘 감싸주도록 하자.

===== 노드들 launching 시 초기 파라미터 값 설정

launch를 통해 노드를 실행할 때도 위와 비슷한 접근법이 있을 것이다.

===== 런타임 때 파라미터 조작

`ros2 param` 커맨드는 런타임 때 파라미터를 조작할 수 있는 일반적인 방법이다. rviz2나 기타 도구로 파라미터를 조작하는 것도 결국 이것의 래퍼일 것으로 보인다.

- `ros2 param list`
- `ros2 param list <node_name>`
- `ros2 param get <node_name> <parameter_name>`
- `ros2 param set <node_name> <parameter_name> <value>`
- `ros2 param delete <node_name> <parameter_name>`
- `ros2 param describe <node_name> <parameter_name>` : 해당 파라미터에 대한 textual description을 보여줄 것이다. 결국 러스트가 취한 자연어 입장에 부합한다고 볼 수 있을 지도...
- `ros2 param dump <node_name>`

파라미터 덤핑은 stdout을 뿜기 때문에 `>` 기호 (redirection: bash에서 stdout을 파일로 저장하는 연산자)를 사용해 파라미터 목록을 편하게 yaml파일로 저장할 수 있다.

```sh
$ ros2 param dump /turtlesim > turtlesim.yaml
```

그렇다면 `ros2 param load <node_name> <parameter_file>`로 파라미터 구성설정을 들고 올 수도 있다는 것이다.

그리고 이러한 작업을 편리하게 지원해주는 도구로 rviz2 같은 게 있다고 보면 될 것이다.

===== rclrs에서의 파라미터

사실 아직도 파라미터를 왜 쓰는지 감이 안 잡힐 수도 있다. 예제 코드를 돌아보던 중 이런 코드가 있었다.

```rust
fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("parameter_demo")?;

    let greeting: MandatoryParameter<Arc<str>> = node
        .declare_parameter("greeting")
        .default("Hello".into())
        .mandatory()?;

    let _subscription =
        node.create_subscription("greet", move |msg: example_interfaces::msg::String| {
            println!("{}, {}", greeting.get(), msg.data);
        })?;

    println!(
        "Ready to provide a greeting. \
        \n\nTo see a greeting, try running\n \
        $ ros2 topic pub greet example_interfaces/msg/String \"data: Alice\"\
        \n\nTo change the kind of greeting, try running\n \
        $ ros2 param set parameter_demo greeting \"Guten tag"\n"
    );
    executor.spin(SpinOptions::default()).first_error()
}
```

```sh
$ ros2 topic pub greet example_interfaces/msg/String "data: Alice"
$ ros2 param set parameter_demo greeting "Guten tag"
```

보다시피 cli 터미널에서 메시지 인터페이스 형식을 명시해서 발행자를 만들어버릴 수도 있다. 물론 이름을 갖는 노드로서 되는 것 같지는 않지만 디버깅 등 용도로 가능한 걸로 보인다.

그리고 파라미터에 대해 보자면 또한 cli 명령어를 통해 get/set을 할 수 있는 걸 볼 수 있습니다. 즉 pid 컨트롤러의 상수 요소 같은걸 외부 프로세스에서 다룰 여지를 제공하는 것입니다.

사실 `topic pub`, `param set` 이런것도 rcl_interfaces에 명세되어 있는 하나의 서비스 형식으로 제공되는 것으로 볼 수 있습니다. 이에 대해서는 rcl_interfaces를 더 공부해보도록 해요

==== Introspection with command line tools

ros2 cli에 대해 알아봅니다. 다음은 모두 ros2 명령어의 서브명령어들입니다.

- action: Introspect/interact with ROS actions
- bag: Record/play a rosbag
- component: Manage component containers
- daemon: Introspect/configure the ROS 2 daemon
- doctor: Check ROS setup for potential issues
- interface: Show information about ROS interfaces
- launch: Run/introspect a launch file
- lifecycle: Introspect/manage nodes with managed lifecycles
- multicast: Multicast debugging commands
- node: Introspect ROS nodes
- param: Introspect/configure parameters on a node
- pkg: Introspect ROS packages
- run: Run ROS nodes
- security: Configure security settings
- service: Introspect/call ROS services
- test: Run a ROS launch test
- topic: Introspect/publish ROS topics
- trace: Tracing tools to get information on ROS nodes execution (only available on Linux)
- wtf: An alias for doctor

한 터미널에서 메시지 발행하기

```sh
$ ros2 topic pub /chatter std_msgs/msg/String "data: Hello world"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello world')

publishing #2: std_msgs.msg.String(data='Hello world')
```

다른 터미널에서 메시지 받기

```sh
$ ros2 topic echo /chatter
data: Hello world

data: Hello world
```

ROS 2 daemon은 백그라운드 discovery service라고 볼 수 있겠다. 이것을 래핑한 것으로 `ros2 node list`, `ros2 topic list` 등의 cli 커맨드가 있다고 보면 된다. 이 daemon은 ROS_DOMAIN_ID와 관련된다.

==== Launch

==== Client Libraries

클라이언트 라이브러리는 각 프로그래밍 언어로 제공되는 API입니다. 또한 language-specific funcionality가 있을 수 있으니 각 언어 문서 api를 확인해보시길 바랍니다. 언어-특화 기능들은 보통 다음들과 같습니다:

- names and namespaces
- time (real or simulated)
- parameters
- console logging
- threading model
- intra-process communication

- rcl
- rclcpp
- rclpy
- rclc
- rclpy

=== Intermediate Concepts

==== ROS_DOMAIN_ID

논리적으로 다른 네트워크가 같은 물리 네트워크를 공유하도록 만들어주는 것. DDS에서 쓰인다.

DDS 참여자 개수에 제한이 있는 것으로 보인다 (120?)

관련해서 domain ID를 UDP Port로 바꾸는 calculator가 있으니 필요시 확인해 보시길 https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html

==== Different ROS 2 middleware vendors

rmw라는 말이 보이면 ros 2 middleware를 의미하는 것이다.

ROS 2 는 DDS/RTPS를 미들웨어 삼아 올려진 미들웨어이다. 이것은 디스커버리, 직렬화, transportation을 지원한다. DDS는 산업 표준으로, 구현체는 vendor에 따라 다르다.

==== Logging and logger configuration

로깅, 디버깅 등을 위한 기능으로 보임..

The logging subsystem in ROS 2 aims to deliver logging messages to a variety of targets, including:
- To the console (if one is attached)
- To log files on disk (if local storage is available)
- To the /rosout topic on the ROS 2 network

severity level : DEBUG, INFO, WARN, ERROR or FATAL

==== Quality of Service settings

노드 사이의 통신을 튜닝할 수 있는 정책이다. 적합한 QoS 정책에 따라, ROS 2는 TCP만큼 안정적이거나 UDP만큼 최선형으로 동작할 수 있으며, 그 사이에는 매우 다양한 state가 존재할 수 있습니다.

===== QoS policies

- history - keep last / keep all
- depth - queue size
- reliability
  - best effort (샘플 일부 손실 가능)
  - reliable (샘플 도착은 보장하는데, 여러번 시도할 것임)
- durability
  - transient local
  - volatile
- deadline - duraction
- lifespan - duration
- liveliness
  - automatic
  - manual by topic
- lease duration - duration

===== QoS profiles

상술한 QoS 정책은 굉장히 복잡하다.. 그리하여 profile이라는 형태로 일종의 템플릿, QoS 상수를 제공하니 이를 잘 활용하도록 하자.

- Default QoS settings for publishers and subscriptions
- Services
- Sensor data
- Parameters
- System default

===== QoS compatiablities

상술한 자유로운 QoS 정책이 모두 허용되는 건 아니다. 살짝 깊이 생각을 해보면, 그 동작 방식에 대해 publisher - subscription이 어떠한 QoS 정책 조합에 대해서만 호환되는지 알 수 있을 것이다. 자세한 표는 사이트에 가서 확인하도록 하자.

이를 통해 QoS profiles 상수를 쓰는 게 편리하다는 것은 알 수 있지만, 당신이 생각하는 더 최적화된 통신 방법이 있다면 manually 구현할 수 있되, 다음 표를 잘 확인하도록 하자.

https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html

===== QoS events

몇몇 QoS 정책들은 그들과 관련된 이벤트를 가질 수 있다.

publisher 관련하여:

- offered deadline missed
- liveliness lost
- offered incompatible QoS

subscription 관련하여:

- requested deadline missed
- liveliness changed
- requested incompatible QoS

==== Executors

실행자는 그것이 실행되는 OS에 기반해서 한개 이상의 스레드를 사용해서 incoming msgs/events에 대하여 구독자, 타이머, 서비스 서버, 액션 서버 등의 콜백을 호출시킨다.

가장 간단한 사용법은 `rclcpp::spin(<node>)`이다. 이렇게 하면 들어오는 메시지, 이벤트를 노드에게 전달한다.

==== Topic statistics

==== Overview and usage of RQt

==== Composition

ros1에서 노드를 직접 executable로 바꾸는 것은 언어를 옮겨가며 패키지를 작성하기에 불편했다. notelet 방식은 공유라이브러리, 콘테이너 프로세스 방식, 런타임 로딩 방식을 사용한다. ros 2에는 이러한 형태가 정착 되었다. Component와 Component Container에 대해 알아보기를 바란다.

==== Cross-compliation

==== ROS 2 Security

==== TF2

tf2는 변환 라이브러리이다.

=== Advanced Concepts

==== The build system

ament 관련

==== Internal ROS 2 interfaces

internal ROS interface는 public C APIs이다. 이것은 client library를 개발하거나 새로운 middleware를 만들고자 할 때 지켜야 할 내용을 보인다.

- user application -> `ros_to_dds` -> DDS
- rclcpp, rclpy, rclrs, ..
  - Exec. with thread
  - IPC
  - Type Adaption
  - language specific features..
- rcl
  - actions
  - time
  - parameters
  - console logging
  - names
  - node lifecycle
- rmw
  - pub/sub with QoS
  - services with QoS
  - discovery
  - graph events
- DDS

`ros_to_dds`란 무엇인가? 사용자가 DDS 공급업체나 미들웨어 기술을 변경하더라도, 사용자 코드에 영향을 최소화시키는 추상화 인터페이스를 달성하기 위한 패키지 범주를 나타내는 것이다. 이것 통해 어떤 코드가 공급업체 이식성을 위반하는지 쉽게 파악할 수 있다고 한다.

static type support, dynamic type support에 대한 다이어그램이 있으니 사이트 가서 확인해보시길 바랍니다 https://docs.ros.org/en/humble/Concepts/Advanced/About-Internal-Interfaces.html

==== ROS 2 middleware implementations

이것은 internal ROS interface (rmw, rcl, rosidl 등)을 구현한 패키지 집합을 칭한다.

== The ROS 2 Project (커뮤니티)

=== Project Governance

ROS PMC(project management committee)에서 관리하는 깃허브 사용자명:

- ament - cmake, index, lint, package, ..
- gazebo-release - gz_cmake_vendor, gz_math_vendor, gz_utils_vendor
- osrf
- ros-infrastructure
- ros-perception - image_common, laser_geometry, point_cloud_transport, pointcloud_to_laserscan
- ros-planning - navigation_msgs
- ros-tooling
- ros-visualization - rqt
- ros
- ros2
  - common_interfaces
  - design (디자인 철학 토론, 블로그)
  - example_interfaces
  - examples
  - geometry2 (TF2)
  - launch, launch_ros
  - ..\_vendor (임베디드 환경과 크로스 컴파일 지원)
  - rcl, rclcpp, rclpy
  - rcl_interfaces
  - rmw (ROS middleware)
각 사용자명 따라서, stack이 있을 수도 있으니 참고
  - ros2, ros2cli, rosbag2
  - rosidl (idl이란 interface description language)
  - rviz
  - urdf (unified robot description format) -> rviz, gazebo 등에 사용됨 https://wiki.ros.org/urdf

== Package Docs

== Related Projects

Gazebo : ROS기반 로봇에 대한 3d 물리 시뮬레이션

=== Large Community Projects

- ros2_control
- Navigation2
- Movelt
- micro-ROS

=== Further Community Projects

https://index.ros.org/?search_packages=true

=== Intel ROS 2 Projects

=== NVIDIA ROS 2 Projects

== RCLCPP

- rclcpp

- rclcpp_lifecycle

- rclcpp_components

- rclcpp_action