import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('saye_bringup')
    pkg_project_localization = get_package_share_directory('saye_localization')
    pkg_project_description = get_package_share_directory('saye_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'saye', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    # Get the absolute path to the world file
    world_file = os.path.join(pkg_project_description, 'worlds', 'saye_world.sdf')
    # Launch gz sim directly to avoid ros_gz_sim launch file adding --follow option
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '--force-version', '8'],
        output='screen'
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'saye.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
    # Bridge ROS topics and Gazebo messages for establishing communication
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'saye',
            '-allow_renaming', 'false',
            '-x', '-255.3',  # Original spawn position on street
            '-y', '-261.2',
            '-z', '0.35',
            '-Y', '1.00'  # Original rotation
        ]
    )
    # Delay spawn to ensure Gazebo is fully initialized and avoid duplicate spawns
    delayed_spawn = TimerAction(period=3.0, actions=[gz_spawn_entity])
    
    # Move camera to view the car and make it follow after simulation starts
    # Wait for model to spawn, then move camera behind it and enable following
    move_camera_script = '''
    sleep 6
    # Position camera behind car once - static position, no jitter
    # Camera stays fixed behind car at spawn location
    # Increased timeout to 5 seconds to avoid timeout warnings
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 5000 --req "pose: {position: {x: -258.0, y: -261.2, z: 5.0}, orientation: {x: 0.0, y: 0.3, z: 0.0, w: 0.955}}" 2>&1 || true
    '''
    move_camera = ExecuteProcess(
        cmd=['bash', '-c', move_camera_script],
        output='screen'
    )
    # Delay camera movement to ensure model is spawned first
    delayed_camera = TimerAction(period=6.5, actions=[move_camera])
    
    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        delayed_spawn,  # Use delayed spawn instead of immediate
        delayed_camera,  # Move camera to follow car
        rviz
    ])
