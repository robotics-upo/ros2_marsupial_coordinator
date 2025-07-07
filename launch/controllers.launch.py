from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():

    uav_control_node = Node(
        package='ros2_marsupial_coordinator',
        executable='uav_control.py',
        name='uav_control_node',
        output='screen'
    )

    ugv_tether_control_node = Node(
        package='ros2_marsupial_coordinator',
        executable='ugv_tether_control.py',
        name='ugv_tether_control_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Controladores individuales empleados en el lanzamiento
    nodes = [
        uav_control_node,
        ugv_tether_control_node,     
    ]

    return LaunchDescription(nodes)
