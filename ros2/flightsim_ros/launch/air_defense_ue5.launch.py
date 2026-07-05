from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_vision = LaunchConfiguration('use_vision_seeker')

    return LaunchDescription([
        DeclareLaunchArgument('use_vision_seeker', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='air_defense'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('flightsim_ros'),
                '/launch/air_defense_vision.launch.py',
            ]),
            launch_arguments={
                'use_vision_seeker': use_vision,
                'scenario': LaunchConfiguration('scenario'),
            }.items(),
        ),
        Node(
            package='flightsim_ros',
            executable='flightsim_ue5_bridge_node',
            name='flightsim_ue5_bridge',
            output='screen',
            parameters=[{
                'camera_width_px': 640,
                'camera_height_px': 480,
                'camera_fov_rad': 0.52,
                'blob_radius_px': 12,
                'background_value': 24,
            }],
            remappings=[
                ('scene_state', '/flightsim/scene_state'),
                ('seeker_camera/image', '/flightsim/seeker_camera/image'),
                ('seeker_camera/camera_info', '/flightsim/seeker_camera/camera_info'),
            ],
        ),
    ])
