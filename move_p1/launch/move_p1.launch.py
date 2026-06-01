import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    pkg_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_viz = get_package_share_directory('turtlebot4_viz')
    pkg_move = get_package_share_directory('move_p1')
    default_map = os.path.join(
        get_package_share_directory('make_map'), 'maps', 'sim_map.yaml')
    # 카메라 PointCloud를 costmap 장애물로 추가한 커스텀 nav2 설정
    nav2_params = os.path.join(pkg_move, 'config', 'nav2_camera.yaml')
    # 자동 초기위치(0,0,0) 설정한 커스텀 localization 설정
    loc_params = os.path.join(pkg_move, 'config', 'localization_auto.yaml')

    # Localization (저장된 지도 + 라이다로 위치 추정)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'localization.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params': loc_params,
        }.items()
    )

    # Nav2 (경로 계획 + 주행)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items()
    )

    # RViz (지도 보기 + 초기 위치 설정)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # move_p1 노드 (CCTV 트리거 → 목표 좌표 이동)
    move_node = Node(
        package='move_p1',
        executable='move_p1',
        name='move_p1_node',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'goal_x': ParameterValue(goal_x, value_type=float),
            'goal_y': ParameterValue(goal_y, value_type=float),
            'goal_yaw': ParameterValue(goal_yaw, value_type=float),
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('goal_x', default_value='1.0'),
        DeclareLaunchArgument('goal_y', default_value='1.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        localization,
        nav2,
        rviz,
        move_node,
    ])
