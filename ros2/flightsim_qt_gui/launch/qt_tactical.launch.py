from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('scenario', default_value='air_defense'),
            DeclareLaunchArgument(
                'start_engagement',
                default_value='true',
                description='Also launch air_defense_dataset engagement publisher',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare('flightsim_ros'),
                        '/launch/air_defense_dataset.launch.py',
                    ]
                ),
                launch_arguments={
                    'scenario': LaunchConfiguration('scenario'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('start_engagement')),
            ),
            Node(
                package='flightsim_qt_gui',
                executable='flightsim_qt_tactical_gui',
                name='flightsim_qt_tactical_gui',
                output='screen',
            ),
        ]
    )
