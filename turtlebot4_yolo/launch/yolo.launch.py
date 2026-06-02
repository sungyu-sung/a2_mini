from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    input_topic = LaunchConfiguration('input_topic')
    conf = LaunchConfiguration('conf')
    viewer = LaunchConfiguration('viewer')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='best.pt 경로 (비우면 패키지 share의 models/best.pt 사용)'),
        DeclareLaunchArgument(
            'input_topic', default_value='/robot2/oakd/rgb/preview/image_raw',
            description='입력 카메라 이미지 토픽'),
        DeclareLaunchArgument(
            'conf', default_value='0.5',
            description='검출 confidence threshold'),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='yolo_viewer(화면 표시) 노드 실행 여부'),

        Node(
            package='turtlebot4_yolo',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'input_topic': input_topic,
                'conf': conf,
            }],
        ),
        Node(
            package='turtlebot4_yolo',
            executable='yolo_viewer',
            name='yolo_viewer',
            output='screen',
            condition=IfCondition(viewer),
        ),
    ])
