import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='/robot2')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot4_navigation'),
                         'launch', 'slam.launch.py')
        ]),
        launch_arguments={'namespace': namespace}.items()
    )

    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('explore_lite'),
                         'launch', 'explore.launch.py')
        ]),
        launch_arguments={'namespace': namespace}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot4_viz'),
                         'launch', 'view_robot.launch.py')
        ]),
        launch_arguments={'namespace': namespace}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot2',
                              description='Robot namespace'),
        slam,
        explore,
        rviz,
    ])
