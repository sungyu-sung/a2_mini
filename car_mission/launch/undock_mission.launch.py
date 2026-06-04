"""undock_mission — topview_undock + undock_navigator 만 실행.

nav2 / localization / rviz 는 사용자가 별도 터미널에서 직접 실행한다.
  (예: ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot2 map:=...
       ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2)

흐름:
  - undock_navigator : 도킹 중 AMCL 초기 pose 발행 → nav 서버 준비되면 ARMED → undock_done 대기
  - topview_undock   : CCTV 로 car 감지 → undock → undock_done 발행
  - undock_navigator : undock_done 받으면 감시포인트로 navigate

실행:  ros2 launch car_mission undock_mission.launch.py
       ros2 launch car_mission undock_mission.launch.py camera_device:=/dev/video4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_device', default_value='/dev/video4'),
        DeclareLaunchArgument('robot_namespace', default_value='/robot2'),
        DeclareLaunchArgument('goal_x', default_value='-2.5104'),
        DeclareLaunchArgument('goal_y', default_value='1.3341'),
        DeclareLaunchArgument('goal_yaw', default_value='-1.37'),
        DeclareLaunchArgument('init_x', default_value='0.0'),     # 도크 위치(map 좌표)
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('startup_delay', default_value='5.0'),
    ]

    ns = LaunchConfiguration('robot_namespace')

    navigator = Node(
        package='car_mission', executable='undock_navigator',
        name='undock_navigator', output='screen',
        parameters=[{
            'robot_namespace': ns,
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_yaw': LaunchConfiguration('init_yaw'),
            'auto_initial_pose': LaunchConfiguration('auto_initial_pose'),
            'startup_delay': LaunchConfiguration('startup_delay'),
            'launch_stacks': False,
            'launch_rviz': False,
        }],
    )

    topview = Node(
        package='car_mission', executable='topview_undock',
        name='topview_undock', output='screen',
        parameters=[{
            'robot_namespace': ns,
            'camera_device': LaunchConfiguration('camera_device'),
        }],
    )

    return LaunchDescription(args + [navigator, topview])
