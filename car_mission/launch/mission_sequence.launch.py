#!/usr/bin/env python3
"""mission_sequence.launch.py — 세 노드를 순차 실행.

흐름(각 노드는 완료 시 스스로 종료 → OnProcessExit 이벤트로 다음 노드 실행):
  1) usb_car_undock     : CCTV(USB캠)로 car 감지 → undock → Nav2로 감시포인트 이동, 도착 시 종료
  2) dori_search_car    : 로봇캠으로 car 탐색·중앙 정렬, 완료(또는 한바퀴 탐색 실패) 시 종료
  3) yolo_depth_detector: car 정렬 + depth 측정 후 목표거리(1m)까지 접근, 도달 시 종료

전제(미리 실행돼 있어야 함): localization, nav2, (RViz에서 2D Pose Estimate).
  → README/안내의 local·nav2 명령 세트 참고.

실행:
  ros2 launch car_mission mission_sequence.launch.py

참고:
  - 각 노드의 자동 종료는 파라미터 exit_on_done(기본 True)로 제어. 단독 디버깅은 False로.
  - OnProcessExit 는 종료코드와 무관하게 '프로세스가 끝나면' 다음을 실행한다(중간 노드가
    Ctrl-C/에러로 죽어도 다음으로 넘어감). 단계별로 따로 돌려 디버깅하려면 ros2 run 으로 실행.
"""
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    undock = Node(
        package='car_mission', executable='usb_car_undock',
        name='usb_car_undock', output='screen')
    search = Node(
        package='car_mission', executable='dori_search_car',
        name='dori_search_car', output='screen')
    detect = Node(
        package='car_mission', executable='yolo_depth_detector',
        name='yolo_depth_detector', output='screen')

    return LaunchDescription([
        LogInfo(msg='[mission] 1/3 usb_car_undock 시작 (CCTV 감지 → undock → 감시포인트 이동)'),
        undock,

        RegisterEventHandler(OnProcessExit(
            target_action=undock,
            on_exit=[
                LogInfo(msg='[mission] usb_car_undock 종료 → 2/3 dori_search_car 시작'),
                search,
            ])),

        RegisterEventHandler(OnProcessExit(
            target_action=search,
            on_exit=[
                LogInfo(msg='[mission] dori_search_car 종료 → 3/3 yolo_depth_detector 시작'),
                detect,
            ])),

        RegisterEventHandler(OnProcessExit(
            target_action=detect,
            on_exit=[
                LogInfo(msg='[mission] yolo_depth_detector 종료 → 미션 완료'),
            ])),
    ])
