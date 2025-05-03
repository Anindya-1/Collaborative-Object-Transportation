from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    minion_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_bringup'),
        "launch",
        "empty_world.launch.py"
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('minion_bringup'),
        'rviz',
        'default.rviz'
    )

    include_minion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(minion_launch_file)
    )

    path_planner_node = Node(
        package="path_planner",
        executable="path_planner"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        include_minion_launch,
        path_planner_node,
        rviz_node
    ])