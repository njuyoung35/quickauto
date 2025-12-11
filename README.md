# 2025년 2학기 자율주행 팀업

## 팀원

- 이상미
- 이환희
- 최주영

## 디렉토리 구성

프로젝트를 다루는 데 있어 미성숙함이 있어 우후죽순으로 레포지토리를 여러 개 만들게 된 것 같습니다.

프로젝트가 종료되어서 한 자리에 모으게 되었습니다.

- adp-repo : 초기에 자율주행 퀴즈 대비, 같이 스터디를 목적으로 하여 공부 내용들을 정리하기 위해 만든 레포입니다.
- adp-ros-packages : 초기에 비공개 레포지토리로, 소스코드를 이곳에서 작성해 프로젝트를 진행하고자 하였습니다.
    - 그러나 너무 방대한 autoware의 모듈들을 재구성한다는 것에 있어 부족함과 조급함을 느끼고, `dongjak_ws` -> `ssu_ws` -> `ssu2_ws` 등으로 워크스페이스 구조를 잘 잡지 못함이 있었습니다.
- quickauto
    - 12월 무렵에 들어, 매번 autoware의 모듈들을 부분적으로 sparse-checkout하는 것의 한계를 느끼고, 아예 `autoware_core`, `autoware_universe` 같이 무겁지만 필수적인 레포를 각자의 환경에 불러오고 (.gitignore로 무시) 부분적으로 가져와 쓰기 편하도록 환경을 구성했습니다.
    - 하지만 애초에 사실상 500개가 넘는 패키지들이 충돌 없이 잘 빌드 되어야 노드와 컴포넌트들이 유기적으로 잘 맞물려 돌아가는 거대 레포지토리였는데, 저의 이런 접근법은 아쉬움이 묻어나왔습니다.
- fina
    - `quickauto`는 패키지를 부분적으로 긁어온 `quick_ws`에서 빌드를 하는 것이었다면, `fina`에서는 `autowarefoundation/autoware`의 메뉴얼을 따라, `autoware`에서 빌드를 하되, `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`과 같이 수정이 필요하다고 생각되어지는 패키지들을 `pop.sh`을 통해 `fina_ws`와 동기화 시키도록 만들었습니다.
    - 이 동기화는 `fina_ws/src`에서 `autoware/src` 방향으로 복사를 '시도'하고, 이후 반드시 역방향으로도 복사를 하도록 만들었습니다.

## fina의 최종 결과
    
이것으로 재현할 수 있는 최종 결과는, `fina/`에서 메뉴얼을 따르고, `. run.sh` 실행 시 rviz2 화면이 뜨며, `SetInitialPose`, `SetGoal`, `RouteTool` (목적지 대략 설정)을 누를 수 있는 것입니다.

이 신호는 `autoware_adapi_adaptors` -> `autoware_default_adapi` -> `mission_planning/route selector` -> `mission_planning/mission_planner`로 향하여 전역 경로 `LaneletRoute`를 생성하도록 합니다.

이 때, autoware의 수많은 알고리즘들이 유효성 및 가치 검사를 하기에, 유효하지 않은 경로일 수도 있으니, 방향과 위치를 정밀하게 주어야 합니다.

다음 스텝은 `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner`가 전달받습니다.

이 연결에서 더 이상 진행이 안 되어서 로그를 분석해서, 강제로 scenario, dynamic_object, operation_mode, occupancy_grid 등을 나름 dummy 데이터라 생각한 0 채워넣기로 보내 구문은 맞췄지만 유효하지 않은 데이터 형식이라고 의미적으로 호환이 되지 않았습니다.
    
그 외의 `/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner` -> `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner` -> `/control/trajectory_follower/controller_node_exe`로 향하고, `/simulator/simple_planning_simulator`로 연결되고 다시 odom, tf를 발행하도록 하는 것은 rqt상 확인했습니다.