import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('gazebo_simulation')

    # tracking_world(평지+파란큐브)로 시뮬 실행. 로봇은 (0,0,0)에서 +X 바라봄
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot4_empty_world.launch.py')),
        launch_arguments={
            'world_file': 'tracking_world',
            'x': '0.0', 'y': '0.0', 'yaw': '0.0',
        }.items()
    )

    # 파란 큐브 cmd_vel 브릿지 (ROS → Ignition VelocityControl)
    cube_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cube_cmd_vel_bridge',
        output='screen',
        arguments=[
            '/model/blue_cube/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ]
    )

    # 추적 노드
    tracker = Node(
        package='tracking',
        executable='tracking',
        name='tracking_node',
        output='screen',
    )

    return LaunchDescription([sim, cube_bridge, tracker])
