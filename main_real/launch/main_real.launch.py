import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 실로봇 설정값
#  ※ P1 좌표/방향은 main_real_node.py 안에 상수로 있음 (TurtleBot4Navigator 사용)
NAMESPACE = 'robot2'
YOLO_MODEL = ''        # TODO: YOLO .pt 경로 (받으면 채움)
YOLO_CLASS = ''        # TODO: 추적할 클래스 이름


def generate_launch_description():
    pkg_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_viz = get_package_share_directory('turtlebot4_viz')
    pkg_move = get_package_share_directory('move_p1')
    pkg_make = get_package_share_directory('make_map')

    # 실로봇 지도 + 설정
    map_yaml = os.path.join(pkg_make, 'maps_real', 'robot2_map.yaml')
    nav2_params = os.path.join(pkg_move, 'config', 'nav2_camera.yaml')
    # 실로봇 독 위치 자동 초기화 설정 (실측값)
    loc_params = os.path.join(
        get_package_share_directory('main_real'), 'config', 'localization_real.yaml')

    ns = f'/{NAMESPACE}'

    # Localization (실로봇 지도)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'localization.launch.py')),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'false',
            'map': map_yaml,
            'params': loc_params,
        }.items()
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'false',
            'params_file': nav2_params,
        }.items()
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'false',
        }.items()
    )

    # YOLO 추적 노드 (처음엔 꺼둠 — main_real이 P1 도착 후 켬)
    tracker = Node(
        package='tracking', executable='tracking_yolo',
        name='tracking_yolo_node', output='screen',
        parameters=[{
            'namespace': NAMESPACE,
            'model_path': YOLO_MODEL,
            'target_class': YOLO_CLASS,
            'enabled': False,
            'use_sim_time': False,
        }]
    )

    # 오케스트레이터 (main_real 자체 노드, TurtleBot4Navigator 사용)
    orchestrator = Node(
        package='main_real', executable='main_real',
        name='main_real_node', output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # localization 먼저 띄우고, nav2/rviz/추적/오케스트레이터는 8초 뒤
    # (동시에 뜨면 lifecycle bond 충돌로 map_server가 활성화 실패하는 문제 방지)
    delayed = TimerAction(period=8.0, actions=[nav2, rviz, tracker, orchestrator])

    return LaunchDescription([localization, delayed])
