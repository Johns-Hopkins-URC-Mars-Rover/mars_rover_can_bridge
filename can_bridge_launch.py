from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS2 SocketCAN Bridge
        Node(
            package='ros2socketcan_bridge',
            executable='ros2socketcan',
            name='can_bridge',
            output='screen',
            parameters=[{
                'can_socket': 'can0'
            }]
        ),
        
        # Motor Control Wrapper
        Node(
            package='mars_rover_can_bridge',
            executable='motor_control',
            name='motor_control_wrapper',
            output='screen'
        ),
    ])
