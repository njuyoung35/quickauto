## init.sh

rustup, colcon-cargo 관련 패키지 설치

## build_ros2_rust.sh

- ros2_rust가 유지하는 ros2_rust_humble.repos에서 의존성 긁어옴. (vcs)
- colcon build까지 수행하는데, 필요에 따라 주석처리 해도 됨.

## vcs_fetcher.sh

- .repos 파일을 파싱해서 레포를 sparse하게 부분적으로 클론할 수 있습니다.
- `alias sparse-clone='script/vcs_fetcher.sh'`를 등록해서 어느 경로이든 편하게 사용합시다.