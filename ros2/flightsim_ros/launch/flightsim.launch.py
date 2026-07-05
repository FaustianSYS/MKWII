from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flightsim_ros',
            executable='flightsim_fdm_node',
            name='flightsim_fdm',
            output='screen',
            parameters=[{
                'dt_sec': 0.01,
                'max_thrust_n': 8000.0,
                'wing_area_m2': 16.0,
                'default_throttle': 0.6,
            }],
        ),
    ])
