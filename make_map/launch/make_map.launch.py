import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_viz = get_package_share_directory('turtlebot4_viz')
    pkg_explore = get_package_share_directory('explore_lite')

    # SLAM (지도 생성)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Nav2 (경로 계획 + 주행 — explore_lite가 목표를 여기로 보냄)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # explore_lite (프론티어 자율 탐색)
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_explore, 'launch', 'explore.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        # 시뮬레이션 기본값: 네임스페이스 없음, sim time 사용
        # 실로봇: namespace:=/robot2 use_sim_time:=false
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace (실로봇은 /robot2)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false'],
                              description='시뮬레이션은 true, 실로봇은 false'),
        slam,
        nav2,
        explore,
        rviz,
    ])
