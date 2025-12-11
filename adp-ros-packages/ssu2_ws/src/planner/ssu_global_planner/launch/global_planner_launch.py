from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    osm_default = TextSubstitution(
    text="/home/misys/adp-ros-packages/ssu_ws/assets/sample-map-planning/lanelet2_map.osm"
)

    return LaunchDescription([
        DeclareLaunchArgument("osm_file", default_value=osm_default),
        DeclareLaunchArgument("frame_id", default_value="map"),

        Node(
            package="ssu_global_planner",
            executable="ssu_global_planner_node",
            output="screen",
            parameters=[{
                "osm_file": LaunchConfiguration("osm_file"),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        )
    ])

