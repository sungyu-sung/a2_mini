"""car_mission 실행 — 현재 실사용 노드 2개.

  - usb_car_undock      : USB CCTV + YOLO(topview) → car 감지 시 undock → Nav2로 감시포인트 이동
  - yolo_depth_detector : 로봇 OAK-D RGB+depth → car/dummy 검출 + car 거리 표시

전제: usb_car_undock 의 Nav2 이동은 Nav2 + localization(map) 이 별도로 실행 중이어야 동작.
      (예: turtlebot4_navigation nav2.launch.py / localization.launch.py)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='car_mission', executable='usb_car_undock',
             name='usb_car_undock', output='screen'),

        Node(package='car_mission', executable='yolo_depth_detector',
             name='yolo_depth_detector', output='screen'),
    ])
