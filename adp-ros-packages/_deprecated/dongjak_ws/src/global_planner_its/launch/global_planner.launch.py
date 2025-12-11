from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_planner_its',
            executable='global_planner_node',
            name='global_planner',
            parameters=['/home/misys/forza_ws/race_stack/src/global_planner_its/config/global_planner.yaml']
        )
    ])

