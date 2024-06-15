import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory for auv_docking
    package_dir = get_package_share_directory('auv_docking')

    # Define the node(s) you want to launch from the auv_docking package
    launch_test = Node(
        package='auv_docking',
        executable='test_node',
        output='screen',
    )

    return LaunchDescription([
        launch_test,
    ])
