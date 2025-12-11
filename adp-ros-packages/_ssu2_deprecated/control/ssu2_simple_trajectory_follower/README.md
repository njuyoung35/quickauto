# ssu2_simple_trajectory_follower

pkg_type: node

## Purpose

간단한 궤적 추종 코드입니다.
추종 궤적과 에고 차량의 운동학을 가지고 계산합니다.

## Inputs

- ~/input/reference_trajectory: `(ssu2_planning_msgs/Trajectory)`
  
  > 따라갈 기준 궤적 \
  >  \- ~/input/current_kinematic_state: `(nav_msgs/Odometry)`
  
  > 현재 차량의 상태 \
  >  \

## Outputs

- output/control_cmd: `(ssu2_control_msgs/Control)`> 생성된 제어 명령 \>  \

## References

- https://github.com/autowarefoundation/autoware_universe/blob/main/control/autoware_trajectory_follower_node/design/simple_trajectory_follower-design.md
