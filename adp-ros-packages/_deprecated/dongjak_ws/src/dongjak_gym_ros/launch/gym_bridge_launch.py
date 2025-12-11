# https://github.com/f1tenth/f1tenth_gym_ros/blob/main/launch/gym_bridge_launch.py

# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('dongjak_gym_ros'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))

    bridge_node = Node(
        package='dongjak_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2,
        arguments=['-d', os.path.join(get_package_share_directory('dongjak_gym_ros'), 'launch', 'gym_bridge.rviz')],
    )
    # map_server_node
    # nav_lifecycle_node

    # https://github.com/ros/robot_state_publisher/tree/humble
    # 로봇의 상태를 tf2에게 발행하는 노드.
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher'
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('dongjak_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    # ld.action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)

    return ld