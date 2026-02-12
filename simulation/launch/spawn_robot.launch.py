#!/usr/bin/env python3
"""
Simplified launch file for spawning robot in Ignition Gazebo.
Works without turtlebot3_gazebo package.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Simulation directory (absolute path)
    simulation_dir = '/home/orinova/Desktop/swacch/hmi/simulation'
    
    # World file path
    world_file = os.path.join(simulation_dir, 'worlds', 'simple_room.world')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch Ignition Gazebo with the world file
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    
    # Add Gazebo
    ld.add_action(gz_sim)
    
    return ld
