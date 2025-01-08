from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tank_odom',
            executable='tank_odom_node',
            name='tank_odom',
            parameters=[{
                'front_wheel_diameter': 0.230,
                'wheel_separation': 0.515,
                'odom_frequency': 50,
                'publish_tf': True
            }],
            output='screen'
        )
    ])