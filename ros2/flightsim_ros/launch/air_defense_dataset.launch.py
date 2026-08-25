"""Phase 0 Mode A — truth publisher for Isaac dataset sidecar.

Runs engagement only (geometric seeker). No vision node, no UE5 bridge.
Isaac Sim subscribes to /flightsim/* topics and renders offline datasets.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='air_defense'),
        DeclareLaunchArgument('dt_sec', default_value='0.01'),
        Node(
            package='flightsim_ros',
            executable='flightsim_engagement_node',
            name='flightsim_engagement',
            output='screen',
            parameters=[{
                'scenario': LaunchConfiguration('scenario'),
                'use_vision_seeker': False,
                'dt_sec': LaunchConfiguration('dt_sec'),
            }],
            remappings=[
                ('scene_state', '/flightsim/scene_state'),
                ('target_state', '/flightsim/target_state'),
                ('missile_state', '/flightsim/missile_state'),
                ('engagement_status', '/flightsim/engagement_status'),
                ('reinitialize', '/flightsim/reinitialize'),
            ],
        ),
    ])
