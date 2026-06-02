import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 실로봇 설정값
NAMESPACE = 'robot2'
P1_X = -2.5104         # 실로봇 지도 기준 P1 좌표 (실측)
P1_Y = 1.3341
P1_YAW = -1.372        # -78.6도 (rad)
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
    # ⚠️ 실로봇 초기위치는 sim과 다름 → 실측한 localization 설정 필요
    #    (지금은 sim 설정 재사용, 실로봇에서 재측정 후 별도 yaml 권장)
    loc_params = os.path.join(pkg_move, 'config', 'localization_auto.yaml')

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

    # YOLO 추적 노드 (처음엔 꺼둠 — main_sim 오케스트레이터가 켬)
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

    # 오케스트레이터 (main_sim 노드 재사용, namespace=robot2)
    orchestrator = Node(
        package='main_sim', executable='main_sim',
        name='main_sim_node', output='screen',
        parameters=[{
            'namespace': NAMESPACE,
            'p1_x': P1_X, 'p1_y': P1_Y, 'p1_yaw': P1_YAW,
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([localization, nav2, rviz, tracker, orchestrator])
