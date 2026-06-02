"""car_mission 통합 실행 (PLACEHOLDER).

띄우는 노드:
  - cctv_detector        (CCTV USB 카메라 + YOLO)
  - turtlebot4_yolo/yolo_detector (로봇 OAK-D RGB + YOLO)  ← 별도 패키지 재사용
  - car_tracker          (검출+depth 융합 → 차량 방위/거리)
  - mission_manager      (상태머신)

전제: Nav2(고정좌표 이동)와 localization(map)이 별도로 실행 중이어야 NAVIGATING 단계가 동작합니다.
      (예: `nav 2`, `loc 2 <map.yaml>` — bashrc 함수)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('car_mission'), 'config', 'mission.yaml')

    return LaunchDescription([
        Node(package='car_mission', executable='cctv_detector',
             name='cctv_detector', output='screen', parameters=[cfg]),

        Node(package='turtlebot4_yolo', executable='yolo_detector',
             name='yolo_detector', output='screen',
             parameters=[{'publish_detections': True}]),

        Node(package='car_mission', executable='car_tracker',
             name='car_tracker', output='screen', parameters=[cfg]),

        Node(package='car_mission', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[cfg]),
    ])
