import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_mower_robot = get_package_share_directory('mower_robot')
    
    # Declare launch arguments
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_mower_robot, 'worlds', 'simple_world.world'),
        description='Path to the Gazebo world file'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'mower_robot',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )
    
    # Robot State Publisher
    try:
        robot_description = open(os.path.join(pkg_mower_robot, 'urdf', 'mower_robot.urdf')).read()
    except FileNotFoundError:
        print("ERROR: mower_robot.urdf not found in", os.path.join(pkg_mower_robot, 'urdf'))
        raise
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Cartographer for SLAM
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            '-configuration_directory', os.path.join(pkg_mower_robot, 'config'),
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('/scan', '/mower_robot/scan'),
            ('/odom', '/mower_robot/odom')
        ]
    )
    
    # Nav2 with TEB planner
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_mower_robot, 'config', 'nav2_params.yaml')
        }.items()
    )
    
    # Initial Pose Publisher
    initial_pose_publisher = Node(
        package='mower_robot',
        executable='initial_pose',
        namespace='mower_robot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Auto navigation node
    auto_nav = Node(
        package='mower_robot',
        executable='auto_nav',
        namespace='mower_robot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_mower_robot, 'config', 'mower_rviz.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_world_file_cmd,
        declare_use_sim_time_cmd,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        cartographer,
        nav2,
        initial_pose_publisher,
        auto_nav,
        rviz
    ])