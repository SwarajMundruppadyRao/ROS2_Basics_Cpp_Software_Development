
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Directory for storing the rosbag output
    bag_dir = os.path.join(os.getcwd(), "results", "bag_output")
    os.makedirs(bag_dir, exist_ok=True)

    return LaunchDescription([
        # Declare frequency argument for the publisher node
        DeclareLaunchArgument(
            'frequency',
            default_value='4',
            description='Publishing frequency for the talker node'),

        # Declare enable_recording argument for conditional bag recording
        DeclareLaunchArgument(
            'enable_recording',
            default_value='True',
            description='Enable or disable ROS 2 bag recording'),

        # Launch the talker node
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            arguments=['talk', '0', '0', '1', '0', '0', '0'],  # Updated to match your command
            parameters=[{'frequency': LaunchConfiguration('frequency')}],
            output='screen'
        ),

        # Conditionally execute ros2 bag record
        ExecuteProcess(
            condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording')])),
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(bag_dir, 'rosbag'), '/chatter', '/service_node'],
            shell=False
        ),
    ])




# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch_ros.actions import Node
# from launch.conditions import IfCondition

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'frequency',
#             default_value='4',
#             description='topic frequency'),
#         DeclareLaunchArgument(
#             'enable_recording',
#             default_value='True'),
#         Node(
#             package='beginner_tutorials',
#             executable='talker',
#             name='talker',
#             arguments = ['talk', 'world', '0', '0', '1', '0', '0', '0'],
#             parameters=[{'frequency': LaunchConfiguration('frequency')}],
#         ),

#         ExecuteProcess(
#             condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording')])),
#             cmd=[['ros2 bag record -o bag_output', '/chatter', '/service_node']],
#             shell=True),
#     ])