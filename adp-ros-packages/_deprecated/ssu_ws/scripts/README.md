# scripts

- 다음 것들은 `universe.yaml`로 수행되는 작업에 이미 통합되어서 개별 사용은 딱히 없을 걸로 보입니다:
    - add_rust_dep.sh
    - new_rust_pkg.sh
    - vcs_fetcher.sh
- add_scripts_path.sh : 단지 이 스크립트 디렉토리 경로를 PATH 환경변수에 추가하는 거..
- 참고로 xsh는 xonsh라는 파이썬+bash라는 스크립트 언어에서 쓰는 파일 형식입니다.
- `sudo apt install xonsh` 하시면 설치 될 겁니다.

## apply-univ.xsh

`universe.yaml`에 명세된 대로 이 워크스페이스 구조에 패키지들을 구성해줍니다.

- 기능
    - repo를 사용해 레포지토리 또는 sparse-checkout을 통한 부분 복사
    - rust/cpp/python? 언어별 기본 패키지 뼈대 생성
        - 의존성 분석해 자동으로 package.xml, Cargo.toml, CMakeLists.txt 작성 (증분방식)
        - 해당 디렉토리의 _README.template.md 파일 참조해 (없으면 template/_README.template.md 복사해줌) README.md 작성.
            - purpose, design, inputs, outputs, parameters, references 등 정보는 애초에 `universe.yaml`에 포함되는 내용.
            - _README.template.md 파일을 읽고 `{{ purpose }}` 같은 문법에 대해 그 내용을 `universe.yaml`에 근거해서 채워넣어주는 방식.
            - README.md는 증분이 아닌, 매번 갈아끼는 방식이니 주의. _README.template.md는 존재한다면 건드리지 않음.

- *증분방식이라고 표기된 것은, 해당 파일을 갈아엎는게 아니라, 파싱해서 부분적으로 추가하는 것을 말합니다.

## read-msgs-univ.xsh

- 원하고자 하는 디렉토리 하위 패키지들을 탐색하며 (재귀방식은 아니고, 1레벨 하위가 무조건 패키지여야함..) 모든 msg, srv, action를 추출해서 변환해서 yaml 으로 저장합니다. (md, txt 형태로도 제공)
- 실제 .msg, .srv, .action 파일을 읽어서 '#' 주석처리 된 걸 분석해서 해당 복합타입이나, 필드 각가에 description으로서 추가해줍니다. (길이가 50보다 길면 자르는 기능이 포함되긴 했으니, 필요시 수정해서 쓰세요)
- 복합타입 자체에 대한 주석이 없으면, 패키지파일의 README.md에서 적당히 긁어서 채워넣습니다. ros2/common_interfaces README.md 기준이라, 다른 패키지그룹에서는 작동 안 할 수도 있어요. (파싱 커스텀 필요)
- 이것이 필요한 이유: common_interfaces 하위 패키지나 autoware_msgs를 사용을 `universe.yaml`에서 명시적으로 명세 안 해도, `apply-univ.xsh`가 분석해서 의존성 추가해줄 수 있도록, 의존성 딕셔너리 만들 때 사용하기 위해서입니다.