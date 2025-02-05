import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "ldr_cntrl"

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    lidar_node = Node(
        package=package_name,
        executable="ldr_cntrl",
        name="ldr_cntrl",
        output="screen"
    )

    return LaunchDescription([gazebo_launch, lidar_node])
