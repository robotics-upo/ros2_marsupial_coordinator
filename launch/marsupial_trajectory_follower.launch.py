from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    urdf_uav_path = os.path.join(
        get_package_share_directory('marsupial_simulator_ros2'),
        'urdf',
        'sjtu_drone.urdf'
    )
    with open(urdf_uav_path, 'r') as infp:
       robot_description_content = infp.read()
    robot_description = {'robot_description': robot_description_content}

    # Lanzamos Configuración de visualización en rviz2
    rviz_config_path = os.path.join(
        get_package_share_directory('ros2_marsupial_coordinator'),
        'rviz',
        'rviz_config.rviz'
    )
 
    # Creamos el nodo de rviz para que se lance 2 segundos con retraso
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    delayed_rviz = TimerAction(
        period=2.0,  # segundos
        actions=[rviz_node]
    )

    return LaunchDescription([

        DeclareLaunchArgument('namespace', default_value='uav'),

        # Argumento de lanzamiento "yaml_filename": se trata del archivo .yaml a pasar como trayectoria
        DeclareLaunchArgument(
            'yaml_filename',
            default_value='trajectory.yaml',
            description='Archivo YAML de la trayectoria'
        ),

        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'pub', '/sjtu_drone/takeoff', 'std_msgs/msg/Empty', '{}', '--once'],
        #     output='screen'
        # ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                #{'frame_prefix': 'uav_'}
            ]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace,
            name='joint_state_publisher_UAV',
            parameters=[robot_description]
        ),
 
        # Nodo publicador de waypoints de la trayectoria para visualizar en Rviz
        Node( 
            package='ros2_marsupial_coordinator',
            executable='wp_publisher.py',
            name='waypoint_pub',
            output='screen',
            arguments=[LaunchConfiguration('yaml_filename')]
        ),

        # Nodo publicador de las posiciones de referencias ugv, uav y tether halladas por el pure pursuit
        Node(
            package='ros2_marsupial_coordinator',
            executable='pure_pursuit_visual_coordinator.py',
            name='trajectory_pure_pursuit',
            output='screen',
            arguments=[LaunchConfiguration('yaml_filename')]
        ),

        # Nodo publicador de transformadas dinámicas
        Node(
            package='ros2_marsupial_coordinator',
            executable='dynamic_tf_pub.py',
            name='dynamic_tf_publisher',
            output='screen',
        ),

        # Nodos de control individuales: UAV y UGV
        Node(
            package='ros2_marsupial_coordinator',
            executable='uav_to_point.py',
            name='uav_node',
            output='screen'
        ),
        Node(
            package='ros2_marsupial_coordinator',
            executable='ugv_to_point.py',
            name='ugv_tether_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
 
        delayed_rviz

    ])
