from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_vision = LaunchConfiguration('use_vision_seeker')

    return LaunchDescription([
        DeclareLaunchArgument('use_vision_seeker', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='air_defense'),
        Node(
            package='flightsim_ros',
            executable='flightsim_engagement_node',
            name='flightsim_engagement',
            output='screen',
            parameters=[{
                'scenario': LaunchConfiguration('scenario'),
                'use_vision_seeker': use_vision,
                'dt_sec': 0.01,
            }],
            remappings=[
                ('scene_state', '/flightsim/scene_state'),
                ('target_state', '/flightsim/target_state'),
                ('missile_state', '/flightsim/missile_state'),
                ('engagement_status', '/flightsim/engagement_status'),
                ('seeker_track', '/flightsim/seeker_track'),
                ('reinitialize', '/flightsim/reinitialize'),
            ],
        ),
        Node(
            package='flightsim_ros',
            executable='flightsim_vision_node',
            name='flightsim_vision',
            output='screen',
            parameters=[{
                'camera_width_px': 640,
                'camera_height_px': 480,
                'camera_fov_rad': 0.52,
                'detection_threshold': 180,
            }],
            remappings=[
                ('seeker_camera/image', '/flightsim/seeker_camera/image'),
                ('missile_state', '/flightsim/missile_state'),
                ('seeker_track', '/flightsim/seeker_track'),
            ],
        ),
    ])
