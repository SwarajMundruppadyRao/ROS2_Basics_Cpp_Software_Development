
"""
@file bag.launch.py
@brief Launch file for ROS 2 bag recording and talker node.

This launch file sets up a ROS 2 environment where a talker node is launched and, optionally, 
ROS 2 bag recording is started based on a launch argument. The recording can be enabled or 
disabled via the 'enable_recording' argument. If enabled, the recording will start and 
automatically stop after 15 seconds.

@details
- The 'enable_recording' launch argument is used to control whether the bag recording is enabled.
- The talker node from the 'beginner_tutorials' package is launched.
- If recording is enabled, all topics are recorded into a bag file located in the 'results' directory.
- The recording process is automatically terminated after 15 seconds using a TimerAction.

@note
Ensure that the 'beginner_tutorials' package is available in your ROS 2 workspace.

@copyright
Copyright (c) 2023 Swaraj. All rights reserved.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    bag_dir = os.path.join(os.getcwd(), "results")
    os.makedirs(bag_dir, exist_ok=True)

    return LaunchDescription([
        # Declare enable_recording argument
        DeclareLaunchArgument(
            'enable_recording',
            default_value='False',
            description='Enable or disable ROS 2 bag recording'),

        # Launch the talker node (example publisher)
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            output='screen'
        ),

        # Conditionally start ros2 bag record for all topics
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                [LaunchConfiguration('enable_recording')])),
            cmd=['ros2', 'bag', 'record', '-a', '-o',
                 os.path.join(bag_dir, 'rosbag')],
            shell=False
        ),

        # Timer to stop the ros2 bag recording after 15 seconds
        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'ros2.bag.record'],
                    shell=False
                )
            ]
        ),
    ])
