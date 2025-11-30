import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ros_tcp_endpoint_node = Node(
    #         package='ros_tcp_endpoint', 
    #         executable='default_server_endpoint', 
    #         name='ros_tcp_endpoint', 
    #         output="screen")

    test = Node(
        package='navigation_pkg', 
        executable='test', 
        name='test_node', 
        output="screen")
    

    planner_node = Node(
        package='navigation_pkg', 
        executable='navigation_node', 
        name='navigation_node', 
        output="screen")


    return LaunchDescription([
        test,
        planner_node,
        # ros_tcp_endpoint_node,
        ])