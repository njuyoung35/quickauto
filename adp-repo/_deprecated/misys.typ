= 프로젝트 구조 알아가기

== misys:forza 도커 이미지로 빌드되는 / 경로 구성 알아가기

```
19G     .
12G     ./usr
  ㄴ 기본 시스템 패키지, 개발 도구, 컴파일러 등이 설치됨
5.4G    ./home
1.7G    ./opt
  ㄴ 1.4GB짜리 nvidia 포함
  ㄴ 343MB짜리 ros 포함
604M    ./root
  ㄴ 그냥 캐시 정도밖에 없음..
129M    ./var
  ㄴ 가변 데이터 저장 (로그, 캐시, 스풀 등)
3.4M    ./etc
1.5M    ./run
740K    ./tmp
12K     ./media
4.0K    ./srv
4.0K    ./mnt
4.0K    ./boot
0       ./sys
0       ./proc
0       ./dev
```

=== /usr에 대하여

```
12G     .
6.5G    ./local
  ㄴ 4.7GB짜리 cuda-12.4 포함
3.7G    ./lib
  ㄴ 2.3GB짜리 x86_64-linux-gnu 포함 (라이브러리, 동적로딩 .so 파일들)
495M    ./share
395M    ./include
129M    ./bin
6.4M    ./sbin
4.0M    ./src
1.6M    ./libexec
4.0K    ./libx32
4.0K    ./lib64
4.0K    ./lib32
4.0K    ./games
```

= /home/misys 구조 알아보기

```
5.4G    .
1.7G    ./.local
  ㄴ ~/.local/lib/python3.10에 파이썬 패키지들 설치되어 있음
1.1G    ./Raceline-Optimization
  ㄴ 이녀석 자체가 python3.8에 의존적이라서 ./raceline/lib/python3.8에 파이썬 패키지들 설치되어 있음 (983MB)
  ㄴ 역할: 글로벌 경로 생성 (minimum curvature, time-optimal, friction map 등..)
963M    ./f1tenth_ws
812M    ./.cache
712M    ./forza_ws
214M    ./.vscode-server
71M     ./.ros
15M     ./f1tenth_gym
204K    ./adp-ros-packages
8.0K    ./.ssh
8.0K    ./.rviz2
8.0K    ./.config
4.0K    ./shared_dir
4.0K    ./.ipython
4.0K    ./.gnupg
```

`colcon build` 실행 시
- `/src` 디렉토리 기준으로 `/build`, `/install`, `/log` 디렉토리 생성 (일반적인 구조)
- 또는 `~/forza_ws/racestack/`에 있는 패키지들 읽고 생성?