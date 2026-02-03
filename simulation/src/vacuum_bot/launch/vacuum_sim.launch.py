#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Use absolute path to simulation directory
    simulation_dir = '/home/orinova/Desktop/swacch/hmi/simulation'
    
    # Path to spawn robot launch file
    spawn_launch = os.path.join(simulation_dir, 'launch', 'spawn_robot.launch.py')
    
    return LaunchDescription([
        # Launch Gazebo with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch)
        ),
        
        # Launch autonomous vacuum node with a delay to allow Gazebo to start
        TimerAction(
            period=5.0,  # Wait 5 seconds for Gazebo to fully load
            actions=[
                Node(
                    package='vacuum_bot',
                    executable='autonomous_vacuum',
                    name='autonomous_vacuum',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
    ])
