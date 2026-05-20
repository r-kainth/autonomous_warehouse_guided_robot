#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('forklift_bot')
    
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.world')
    
    # Process xacro file
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Gazebo (like robo_test)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'forklift', '-z', '0.2'],
        output='screen'
    )
    
    # Controller spawners (like robo_test)
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    spawn_fork_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fork_controller'],
        output='screen'
    )
    
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )
    
    # Box Detector
    box_detector = Node(
        package='forklift_bot',
        executable='box_detector',
        output='screen',
        parameters=[{'target_color': 'red', 'min_area': 500, 'use_sim_time': True}]
    )
    
    # Forklift Controller
    forklift_controller = Node(
        package='forklift_bot',
        executable='forklift_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Mission Controller
    mission_controller = Node(
        package='forklift_bot',
        executable='mission_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[spawn_joint_state_broadcaster, spawn_fork_controller, spawn_diff_drive],
            )
        ),
        TimerAction(period=5.0, actions=[box_detector]),
        TimerAction(period=5.0, actions=[forklift_controller]),
        TimerAction(period=6.0, actions=[mission_controller])
    ])
