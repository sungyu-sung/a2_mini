import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('gazebo_simulation')
    pkg_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_viz = get_package_share_directory('turtlebot4_viz')
    pkg_move = get_package_share_directory('move_p1')
    pkg_make = get_package_share_directory('make_map')

    map_yaml = os.path.join(pkg_make, 'maps', 'sim_map.yaml')
    nav2_params = os.path.join(pkg_move, 'config', 'nav2_camera.yaml')
    loc_params = os.path.join(pkg_move, 'config', 'localization_auto.yaml')

    # 1) 시뮬 (방 구조 empty + 파란 큐브)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot4_empty_world.launch.py')),
        launch_arguments={'world_file': 'empty'}.items()
    )

    # 2) 큐브 cmd_vel 브릿지 (wasd 제어용)
    cube_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='cube_cmd_vel_bridge', output='screen',
        arguments=['/model/blue_cube/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist']
    )

    # 3) Localization (자동 초기위치)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'localization.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'map': map_yaml, 'params': loc_params,
        }.items()
    )

    # 4) Nav2 (카메라 costmap)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'params_file': nav2_params,
        }.items()
    )

    # 5) RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_viz, 'launch', 'view_robot.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 6) 추적 노드 (처음엔 꺼둠 — main_sim이 켬)
    tracker = Node(
        package='tracking', executable='tracking',
        name='tracking_node', output='screen',
        parameters=[{'enabled': False, 'use_sim_time': True}]
    )

    # 7) 오케스트레이터
    orchestrator = Node(
        package='main_sim', executable='main_sim',
        name='main_sim_node', output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim, cube_bridge, localization, nav2, rviz, tracker, orchestrator,
    ])
