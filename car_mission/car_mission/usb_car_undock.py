#!/usr/bin/env python3

import os
from math import cos, sin

import cv2
import rclpy
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class UsbCarUndock(Node):
    def __init__(self):
        super().__init__('usb_car_undock')

        # ---- parameters (PC/미션마다 바뀌는 값들) ----
        # 외장 USB캠(Jieli USB Composite Device). by-id 경로라 재연결/리부팅해도 인덱스 안 바뀜.
        # 내장 HP 캠(/dev/video4 계열)과 헷갈리지 않게 고정.
        self.declare_parameter(
            'camera_device',
            '/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0')
        self.declare_parameter('model_path', '')                # 비우면 share/models/topview_best.pt
        self.declare_parameter('target_class', 'car')
        self.declare_parameter('confidence', 0.7)
        self.declare_parameter('required_hits', 5)              # N프레임 연속 감지 시 트리거
        self.declare_parameter('robot_namespace', '/robot2')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_x', -2.5104)               # 감시포인트
        self.declare_parameter('goal_y', 1.3341)
        self.declare_parameter('goal_yaw', -1.37)
        self.declare_parameter('show', True)                    # cv2 창 표시
        # 이 USB캠은 YUYV 풀해상도면 USB 대역폭 부족으로 select() timeout 발생 → MJPG로 강제
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        # True면 Nav2 목표 도착 후 노드 자동 종료(다음 단계로 넘어가기 위함). 단독 테스트는 False.
        self.declare_parameter('exit_on_done', True)

        camera_device = self.get_parameter('camera_device').value
        model_path = self.get_parameter('model_path').value
        self.target_class = self.get_parameter('target_class').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.required_hits = int(self.get_parameter('required_hits').value)
        self.goal_frame = self.get_parameter('goal_frame').value
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.show = bool(self.get_parameter('show').value)

        ns = self.get_parameter('robot_namespace').value.strip('/')
        prefix = f'/{ns}' if ns else ''
        dock_status_topic = f'{prefix}/dock_status'
        undock_action = f'{prefix}/undock'
        nav_action = f'{prefix}/navigate_to_pose'

        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('car_mission'), 'models', 'topview_best.pt')

        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        fourcc = self.get_parameter('fourcc').value
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('frame_width').value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('frame_height').value))
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open camera: {camera_device}')

        self.model = YOLO(model_path)
        self.is_docked = False
        self.hit_count = 0
        self.undock_sent = False
        self.nav_sent = False
        self.want_nav = False   # undock 완료 후 nav 보내야 함 (서버 준비되면 timer에서 전송)
        self.exit_on_done = bool(self.get_parameter('exit_on_done').value)
        self.finished = False   # True가 되면 main 루프가 spin 종료(프로세스 exit)
        self.nav_retry_at = 0.0  # goal 거부 시 이 시각 이후에만 재시도(0.1s 폭주 방지)

        self.create_subscription(
            DockStatus,
            dock_status_topic,
            self.dock_status_callback,
            10,
        )
        self.undock_client = ActionClient(self, Undock, undock_action)
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f'usb_car_undock 시작 — camera={camera_device}, model={os.path.basename(model_path)}, '
            f'goal=({self.goal_x}, {self.goal_y}, {self.goal_yaw})')

    def dock_status_callback(self, msg):
        self.is_docked = msg.is_docked

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera frame read failed')
            return

        result = self.model.predict(frame, conf=self.confidence, verbose=False)[0]
        car_detected = self.has_target(result)
        self.hit_count = self.hit_count + 1 if car_detected else 0
        if self.show:
            cv2.imshow('usb camera', result.plot())
            cv2.waitKey(1)

        if self.can_undock():
            self.send_undock()

        # undock 완료 후, navigate_to_pose 서버가 준비되면 nav goal 전송 (서버 발견까지 재시도)
        # 거부(reject)되면 nav_retry_at 이후에만 재시도 → 0.1s 로그 폭주 방지
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.want_nav and not self.nav_sent and now >= self.nav_retry_at:
            if self.nav_client.server_is_ready():
                self.send_nav_goal()
            else:
                self.get_logger().warn('navigate_to_pose 서버 대기 중... (Nav2 실행/active 확인)',
                                       throttle_duration_sec=2.0)

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
        self.want_nav = True   # 실제 전송은 timer에서 서버 준비 확인 후

    def send_nav_goal(self):
        if self.nav_sent:
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.make_goal_pose()

        self.nav_sent = True
        self.get_logger().info(
            f'Navigating to x={self.goal_x}, y={self.goal_y}, yaw={self.goal_yaw}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_sent = False
            self.nav_retry_at = self.get_clock().now().nanoseconds * 1e-9 + 2.0
            self.get_logger().warn(
                'Navigate goal 거부됨 → 2초 후 재시도. Nav2 active 여부 확인: '
                'ros2 lifecycle get /robot2/bt_navigator (active 여야 함), '
                'ros2 action list | grep navigate_to_pose',
                throttle_duration_sec=2.0)
            return

        self.get_logger().info('Navigate goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.get_logger().info(f'Navigate finished: status={future.result().status}')
        if self.exit_on_done:
            self.get_logger().info('감시포인트 도착 → usb_car_undock 종료(다음 단계로)')
            self.finished = True

    def make_goal_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.orientation.z = sin(self.goal_yaw / 2.0)
        pose.pose.orientation.w = cos(self.goal_yaw / 2.0)
        return pose

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCarUndock()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
