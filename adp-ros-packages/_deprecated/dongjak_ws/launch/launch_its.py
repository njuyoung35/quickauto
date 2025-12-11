from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

ws = os.path.expanduser('~/forza_ws/race_stack')
stack_master = os.path.join(ws, 'src', 'stack_master', 'launch', 'head_to_head_launch.xml')

map_loader_params = os.path.join(ws, 'src', 'its_map_loader', 'config', 'its_map_loader.yaml')
planner_params    = os.path.join(ws, 'src', 'global_planner_its', 'config', 'global_planner.yaml')

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack_master),
            launch_arguments={'sim': 'true'}.items()
        ),
        Node(
            package='its_map_loader',
            executable='its_map_loader_node',
            name='its_map_loader',
            parameters=[map_loader_params]
        ),
        Node(
            package='global_planner_its',
            executable='global_planner_node',
            name='global_planner',
            parameters=[planner_params]
        ),
    ])

