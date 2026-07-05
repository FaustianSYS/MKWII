from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flightsim_ros',
            executable='flightsim_engagement_node',
            name='flightsim_engagement',
            output='screen',
            parameters=[{
                'dt_sec': 0.01,
                'target_speed_mps': 45.0,
                'target_rng_seed': 12345,
                'missile_navigation_gain': 5.0,
                'missile_max_speed_mps': 700.0,
            }],
        ),
    ])
