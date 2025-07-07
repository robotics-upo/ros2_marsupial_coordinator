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

    world_arg = DeclareLaunchArgument('world', default_value='stage_1.world')
    pos_x_arg = DeclareLaunchArgument('pos_x', default_value='0.0')
    pos_y_arg = DeclareLaunchArgument('pos_y', default_value='0.0')
    pos_z_arg = DeclareLaunchArgument('pos_z', default_value='0.0')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='uav')
    yaml_filename_arg = DeclareLaunchArgument('yaml_filename', default_value='trajectory.yaml')

    world = LaunchConfiguration('world')
    pos_x = LaunchConfiguration('pos_x')
    pos_y = LaunchConfiguration('pos_y')
    pos_z = LaunchConfiguration('pos_z')
    namespace = LaunchConfiguration('namespace')
    yaml_filename = LaunchConfiguration('yaml_filename')

    marsupial_simulation_launch_path = os.path.join(
        get_package_share_directory('marsupial_simulator_ros2'),
        'launch',
        'marsupial_simulation.launch.py'      # CON TEATRO
        #'marsupial_simulation_copy.launch.py' # SIN TEATRO
    )

    marsupial_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(marsupial_simulation_launch_path),
        launch_arguments={
            'world': world,
            'pos_x': pos_x,
            'pos_y': pos_y,
            'pos_z': pos_z,
        }.items()
    )

    urdf_uav_path = os.path.join(
        get_package_share_directory('marsupial_simulator_ros2'),
        'urdf',
        'sjtu_drone.urdf'
    )
    with open(urdf_uav_path, 'r') as infp:
        robot_description_content = infp.read()
    robot_description = {'robot_description': robot_description_content}

    rviz_config_path = os.path.join(
        get_package_share_directory('ros2_marsupial_coordinator'),
        'rviz',
        'rviz_config.rviz'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher_node',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        name='joint_state_publisher_node',
        output='screen'
    )

    waypoint_publisher_node = Node(
        package='ros2_marsupial_coordinator',
        executable='wp_publisher_catenary.py',
        name='waypoint_publisher_node',
        output='screen',
        arguments=[yaml_filename]
    )

    pure_pursuit_node = Node(
        package='ros2_marsupial_coordinator',
        executable='pure_pursuit_adaptative.py',  # LOOKAHEAD ADAPTATIVO
        name='pure_pursuit_node',
        output='screen',
        arguments=[yaml_filename]
    )

    dynamic_tf_publisher_node = Node(
        package='ros2_marsupial_coordinator',
        executable='dynamic_tf_pub.py',
        name='dynamic_tf_publisher_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    delayed_rviz_node = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    takeoff_pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/sjtu_drone/takeoff', 'std_msgs/msg/Empty', '{}', '--once'],
        output='screen'
    )

    wait = TimerAction(period=5.0, actions=[
        robot_state_publisher_node,
        joint_state_publisher_node,
        waypoint_publisher_node,
        pure_pursuit_node,
        dynamic_tf_publisher_node,

        delayed_rviz_node])
    
    wait_takeoff = TimerAction(period=17.0, actions=[takeoff_pub])

    nodes = [
        world_arg,
        pos_x_arg,
        pos_y_arg,
        pos_z_arg,
        namespace_arg,

        yaml_filename_arg,

        marsupial_simulation_launch,

        wait,

        wait_takeoff,
        
    ]

    return LaunchDescription(nodes)
