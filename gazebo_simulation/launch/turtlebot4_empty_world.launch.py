import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WORLD = 'warehouse'
ROBOT = 'turtlebot4'


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_simulation')
    pkg_bringup = get_package_share_directory('turtlebot4_ignition_bringup')

    world_path = os.path.join(pkg, 'worlds', 'empty')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_bringup, 'launch', 'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
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

    return LaunchDescription([sim, lidar_bridge, camera_bridge])
