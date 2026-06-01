import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

WORLD = 'warehouse'   # 두 world 파일 모두 내부 world name이 'warehouse' (브릿지 토픽 경로용)
ROBOT = 'turtlebot4'


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_simulation')
    pkg_bringup = get_package_share_directory('turtlebot4_ignition_bringup')

    world_file = LaunchConfiguration('world_file')

    # turtlebot4_ignition.launch.py가 .sdf를 붙이므로 확장자 없이 경로 전달
    world_path = PathJoinSubstitution([pkg, 'worlds', world_file])

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_bringup, 'launch', 'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            f'/world/{WORLD}/model/{ROBOT}/link/rplidar_link/sensor/rplidar/scan'
            '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        remappings=[
            (f'/world/{WORLD}/model/{ROBOT}/link/rplidar_link/sensor/rplidar/scan', '/scan')
        ]
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            f'/world/{WORLD}/model/{ROBOT}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            (f'/world/{WORLD}/model/{ROBOT}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image',
             '/oakd/rgb/preview/image_raw')
        ]
    )

    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_bridge',
        output='screen',
        arguments=[
            f'/world/{WORLD}/model/{ROBOT}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image'
            '@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        remappings=[
            (f'/world/{WORLD}/model/{ROBOT}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image',
             '/oakd/rgb/preview/depth')
        ]
    )

    # depth → PointCloud2 (Nav2 costmap이 장애물로 등록)
    points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='points_bridge',
        output='screen',
        arguments=[
            f'/world/{WORLD}/model/{ROBOT}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points'
            '@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            (f'/world/{WORLD}/model/{ROBOT}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points',
             '/oakd/rgb/preview/depth/points')
        ]
    )

    return LaunchDescription([
        # world_file: 'empty'(방 구조, 기본) 또는 'flat_obstacles'(평지+장애물2개)
        DeclareLaunchArgument('world_file', default_value='empty',
                              description="world 파일명 (empty / flat_obstacles)"),
        DeclareLaunchArgument('x', default_value='3.5'),
        DeclareLaunchArgument('y', default_value='3.78'),
        DeclareLaunchArgument('yaw', default_value='1.57'),
        sim,
        lidar_bridge,
        camera_bridge,
        depth_bridge,
        points_bridge,
    ])
