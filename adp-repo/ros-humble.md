[https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)

## tutorials

### cli tools

#### 환경 설정

ros2는 `workspace`와 `쉘 환경` 개념을 결합하는 데 의존한다.

코어 ros2 워크스페이스는 `underlay`라고 불리운다.

`source /opt/ros/humble/setup.bash`

DDS 커뮤니케이션에서 다른 논리 네트워크가 같은 물리 네트워크를 공유하는 것의 주된 메커니즘은 같은 `DOMAIN_ID`를 사용해서이다.

`echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc`를 통해 `.bashrc`에 도메인 id를 저장하도록 하자.

`ROS_LOCALHOST_ONLY=1` 환경변수를 설정하면 로컬 호스트에서만 토픽, 서비스 액션 등을 통신할 수 있다.

#### turtlesim, ros2, rqt

`ros2 pkg executeables turtlesim`로 `turtlesim` 패키지의 하위 실행단위를 검색해보자.

`ros2 run turtlesim turtlesim_node`로 터틀시뮬레이션을 실행해보자.

```
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

`rqt`를 사용해서 `Plugins > Services > Service Caller`를 실행.

Request와 Response를 확인할 수 있다.

#### understanding nodes

#### understanding topics

#### understanding services

#### understanding parameters

#### understanding actions

#### 요약 및 rqt\_console

`ros_tutorials` 깃허브 레포지토리를 보면 `turtlesim` 패키지와, `turtlesim_msgs` 패키지가 나뉘어 있다.

전자는 알고리즘 cpp 코드, 후자는 `.srv` 형식 파일들만 포함한 명세서로 보인다. 이것은 비단 이 패키지 뿐만 아니라, 다른 패키지 묶음에 대해서도 적용되는 관행으로 보인다.

전자를 노트 패키지, 후자를 인터페이스 패키지라고 부를 수 있는 것으로 보인다(챗봇피셜)

후자는 builtool 의존성을 `ament_cmake`와 `rosidl_default_generators`에 걸고 있는데 `rosidl_default_generators`는 `.srv`, `.msg` 파일을 읽어들여 c++, python 등의 템플릿 코드 (c++의 경우 .hpp 파일)를 생성하는 로직을 갖고 있다.
