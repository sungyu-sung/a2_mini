#!/usr/bin/env python3

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


CAMERA_DEVICE = '/dev/video2'
MODEL_PATH = '/home/rokey/a2_mini_ws/src/turtlebot4_image/turtlebot4_image/best.pt'
TARGET_CLASS = 'car'
CONFIDENCE = 0.7
REQUIRED_HITS = 5
DOCK_STATUS_TOPIC = '/robot2/dock_status'
UNDOCK_ACTION = '/robot2/undock'
NAV_ACTION = '/robot2/navigate_to_pose'
GOAL_FRAME = 'map'
GOAL_X = -2.5104
GOAL_Y = 1.3341
GOAL_YAW = -1.37


class UsbCarUndock(Node):
    def __init__(self):
        super().__init__('usb_car_undock')

        self.cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open camera: {CAMERA_DEVICE}')

        self.model = YOLO(MODEL_PATH)
        self.is_docked = False
        self.hit_count = 0
        self.undock_sent = False
        self.nav_sent = False

        self.create_subscription(
            DockStatus,
            DOCK_STATUS_TOPIC,
            self.dock_status_callback,
            10,
        )
        self.undock_client = ActionClient(self, Undock, UNDOCK_ACTION)
        self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION)
        self.create_timer(0.1, self.timer_callback)

    def dock_status_callback(self, msg):
        self.is_docked = msg.is_docked

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera frame read failed')
            return

        result = self.model.predict(frame, conf=CONFIDENCE, verbose=False)[0]
        car_detected = self.has_target(result)
        self.hit_count = self.hit_count + 1 if car_detected else 0
        cv2.imshow('usb camera', result.plot())
        cv2.waitKey(1)

        if self.can_undock():
            self.send_undock()

    def has_target(self, result):
        for box in result.boxes:
            class_id = int(box.cls[0])
            if result.names[class_id] == TARGET_CLASS:
                return True
        return False

    def can_undock(self):
        return (
            self.is_docked
            and not self.undock_sent
            and self.hit_count >= REQUIRED_HITS
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
        self.send_nav_goal()

    def send_nav_goal(self):
        if self.nav_sent:
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Navigate action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.make_goal_pose()

        self.nav_sent = True
        self.get_logger().info(f'Navigating to x={GOAL_X}, y={GOAL_Y}, yaw={GOAL_YAW}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigate goal rejected')
            self.nav_sent = False
            return

        self.get_logger().info('Navigate goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.get_logger().info(f'Navigate finished: status={future.result().status}')

    def make_goal_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = GOAL_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = GOAL_X
        pose.pose.position.y = GOAL_Y
        pose.pose.orientation.z = sin(GOAL_YAW / 2.0)
        pose.pose.orientation.w = cos(GOAL_YAW / 2.0)
        return pose

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCarUndock()
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
