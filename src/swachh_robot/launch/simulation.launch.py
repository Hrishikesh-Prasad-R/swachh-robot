import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo with custom world, spawn robot, and start autonomous navigator."""

    # Get package directories
    pkg_swachh_robot = get_package_share_directory('swachh_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # World file path
    world_file = os.path.join(pkg_swachh_robot, 'worlds', 'simple_room.world')

    # Robot model path
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    robot_sdf = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        f'turtlebot3_{turtlebot3_model}',
        'model.sdf'
    )

    # Launch Gazebo Server with ROS plugins
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', 
             '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    # Launch Gazebo Client (GUI)
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn the TurtleBot3 (delayed to allow Gazebo to start)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', f'turtlebot3_{turtlebot3_model}',
                    '-file', robot_sdf,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )

    # Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Autonomous Navigator (delayed to allow robot to spawn)
    autonomous_navigator = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='swachh_robot',
                executable='autonomous_navigator',
                name='autonomous_navigator',
                output='screen',
                parameters=[{
                    'obstacle_distance': 0.5,
                    'forward_speed': 0.2,
                    'turn_speed': 0.5,
                    'turn_duration': 2.0
                }]
            )
        ]
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher,
        spawn_robot,
        autonomous_navigator,
    ])
