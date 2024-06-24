from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument for log level
        # DeclareLaunchArgument(
        #     'log_level',
        #     default_value='INFO', # Default for other (all) nodes
        #     description='Logging verbosity level',
        # ),
        Node (
            package='robot_patrol',
            executable='robot_patrol_node',
            output='screen',
            # parameters=[{'log_level' : 'debug'}],
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
        ),
    ])