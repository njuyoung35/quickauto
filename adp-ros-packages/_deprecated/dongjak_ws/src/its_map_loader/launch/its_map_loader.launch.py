from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='its_map_loader',
            executable='its_map_loader_node',
            name='its_map_loader',
            parameters=['/home/misys/forza_ws/race_stack/src/its_map_loader/config/its_map_loader.yaml']
        )
    ])

