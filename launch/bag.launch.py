from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
import datetime

def generate_launch_description():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
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
            condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording')])),
            cmd=['ros2', 'bag', 'record', '-a', '-o', os.path.join(bag_dir, 'rosbag')],
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