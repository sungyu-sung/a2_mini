#!/usr/bin/env python3
"""topview_undock — 탑뷰(USB CCTV) 카메라로 car 감지 시 TurtleBot4 undock.

usb_car_undock.py 를 두 노드로 분리한 것 중 [감지 + undock] 담당.
  - PC USB CCTV(topview) 로 YOLO 추론, car 가 required_hits 프레임 연속 감지되고
    로봇이 도킹 상태(is_docked)면 Undock 액션 전송.
  - undock 완료 시 {ns}/undock_done(Bool, latched) 를 발행 → undock_navigator 가
    받아 Nav2 스택을 기동하고 목표로 이동한다.

Nav2/localization/rviz/목표이동은 이 노드가 관여하지 않는다(undock_navigator 담당).
"""
import os

import cv2
import rclpy
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class TopviewUndock(Node):
    def __init__(self):
        super().__init__('topview_undock')

        # ---- parameters ----
        self.declare_parameter('camera_device', '/dev/video4')  # USB 외장캠 (노트북 내장 HP캠은 video0~3)
        self.declare_parameter('model_path', '')                # 비우면 share/models/topview_best.pt
        self.declare_parameter('target_class', 'car')
        self.declare_parameter('confidence', 0.7)
        self.declare_parameter('required_hits', 5)              # N프레임 연속 감지 시 트리거
        self.declare_parameter('robot_namespace', '/robot2')
        self.declare_parameter('show', True)                    # cv2 창 표시

        camera_device = self.get_parameter('camera_device').value
        model_path = self.get_parameter('model_path').value
        self.target_class = self.get_parameter('target_class').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.required_hits = int(self.get_parameter('required_hits').value)
        self.show = bool(self.get_parameter('show').value)

        ns = self.get_parameter('robot_namespace').value.strip('/')
        prefix = f'/{ns}' if ns else ''
        dock_status_topic = f'{prefix}/dock_status'
        undock_action = f'{prefix}/undock'

        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('car_mission'), 'models', 'topview_best.pt')

        self.cap = cv2.VideoCapture(camera_device)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open camera: {camera_device}')

        self.model = YOLO(model_path)
        self.is_docked = False
        self.hit_count = 0
        self.undock_sent = False

        # undock 완료 신호 (latched: 늦게 뜬 navigator 도 마지막 값 수신하도록 TRANSIENT_LOCAL)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.undock_done_pub = self.create_publisher(Bool, f'{prefix}/undock_done', latched)

        # 지민님 검증본과 동일하게 dock_status 는 기본 QoS(reliable, depth 10) 로 구독
        self.create_subscription(DockStatus, dock_status_topic, self.dock_status_callback, 10)
        self.dock_status_seen = False
        self.undock_client = ActionClient(self, Undock, undock_action)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f'topview_undock 시작 — camera={camera_device}, model={os.path.basename(model_path)}, '
            f'undock_done→{prefix}/undock_done')

    def dock_status_callback(self, msg):
        self.is_docked = msg.is_docked
        self.dock_status_seen = True

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera frame read failed')
            return

        result = self.model.predict(frame, conf=self.confidence, verbose=False)[0]
        car_detected = self.has_target(result)
        self.hit_count = self.hit_count + 1 if car_detected else 0
        if self.show:
            cv2.imshow('topview camera', result.plot())
            cv2.waitKey(1)

        # 진단 로그: 왜 undock 안 되는지 한눈에 (1초마다)
        self.get_logger().info(
            f'car={car_detected} hits={self.hit_count}/{self.required_hits} '
            f'dock_status수신={self.dock_status_seen} is_docked={self.is_docked} '
            f'undock_sent={self.undock_sent}',
            throttle_duration_sec=1.0)

        if self.can_undock():
            self.send_undock()

    def has_target(self, result):
        for box in result.boxes:
            class_id = int(box.cls[0])
            if result.names[class_id] == self.target_class:
                return True
        return False

    def can_undock(self):
        return (
            self.is_docked
            and not self.undock_sent
            and self.hit_count >= self.required_hits
        )

    def send_undock(self):
        if not self.undock_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Undock action server not available')
            return

        self.undock_sent = True
        self.get_logger().info('Car detected. Sending undock goal.')
        future = self.undock_client.send_goal_async(Undock.Goal())
        future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Undock goal rejected')
            self.undock_sent = False
            return

        self.get_logger().info('Undock goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        self.get_logger().info(f'Undock finished: status={future.result().status}')
        self.undock_done_pub.publish(Bool(data=True))
        self.get_logger().info('undock_done(True) 발행 → undock_navigator 가 Nav2 기동')

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TopviewUndock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
