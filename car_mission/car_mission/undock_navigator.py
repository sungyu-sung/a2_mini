#!/usr/bin/env python3
"""undock_navigator — Nav2 스택을 미리 띄워 도킹 중 localize 시키고, undock 완료 시 목표로 이동.

usb_car_undock.py 분리본 중 [Nav2 + 자율주행] 담당.

⚠️ 검증된 시퀀스(지민님 최종본 운영방식)를 그대로 재현한다:
  지민님 환경은 localization 이 undock '전에' 떠 있었고(README: Nav2+localization 별도 실행 전제),
  로봇이 도킹된 상태에서 AMCL 이 먼저 수렴 → undock(odom 이 이동 추적) → navigate 순서였다.
  그래서 이 노드도 '시작하자마자' 스택을 띄우고 도킹 중에 초기 pose 를 박는다.
  (undock 후에 localization 을 띄우면 로봇이 이미 도크에서 빠져나온 뒤라 초기 pose 오차가 생김)

흐름:
  [노드 시작] launch_stacks → localization+nav2 / launch_rviz → rviz 기동 (로봇 도킹 중)
     → (auto_initial_pose) AMCL 초기 pose(도크=map 0,0,0) 발행 → startup_delay 동안 수렴
     → ARMED (대기) ─ topview_undock 의 {ns}/undock_done(True) 수신 ─→ navigate_to_pose goal 전송

상태머신(0.5s tick):
  BOOT → SETTLE(초기 pose 발행·수렴 대기) → ARMED ─(undock_done)→ SEND_GOAL → NAVIGATING → DONE

옵션:
  - RViz 에서 수동으로 "2D Pose Estimate" 를 찍고 싶으면 auto_initial_pose:=False (창만 띄움).
  - 스택을 외부에서 따로 띄우는 운영이면 launch_stacks:=False / launch_rviz:=False.
"""
import subprocess
from math import cos, sin

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool


class UndockNavigator(Node):
    def __init__(self):
        super().__init__('undock_navigator')

        # ---- parameters ----
        self.declare_parameter('robot_namespace', '/robot2')
        self.declare_parameter('map_path', '/home/sungyu/a2_mini_ws/src/map/robot2_map.yaml')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_x', -2.5104)               # 감시포인트
        self.declare_parameter('goal_y', 1.3341)
        self.declare_parameter('goal_yaw', -1.37)

        self.declare_parameter('launch_stacks', False)          # nav2/local 은 사용자가 별도 터미널에서 실행
        self.declare_parameter('launch_rviz', False)            # rviz 도 별도 터미널
        self.declare_parameter('startup_delay', 8.0)            # 스택 기동 후 AMCL 수렴 대기(초)

        self.declare_parameter('auto_initial_pose', True)       # 도킹 중 AMCL 초기 pose 자동 발행
        self.declare_parameter('init_x', 0.0)                   # 도크 = map (0,0,0) 가정
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)

        ns = self.get_parameter('robot_namespace').value.strip('/')
        self.prefix = f'/{ns}' if ns else ''
        self.map_path = self.get_parameter('map_path').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.launch_stacks = bool(self.get_parameter('launch_stacks').value)
        self.launch_rviz = bool(self.get_parameter('launch_rviz').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.auto_initial_pose = bool(self.get_parameter('auto_initial_pose').value)
        self.init_x = float(self.get_parameter('init_x').value)
        self.init_y = float(self.get_parameter('init_y').value)
        self.init_yaw = float(self.get_parameter('init_yaw').value)

        # undock_done: topview_undock 과 동일한 latched QoS(늦게 떠도 마지막 값 수신)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, f'{self.prefix}/undock_done',
                                 self.undock_done_callback, latched)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, f'{self.prefix}/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, f'{self.prefix}/navigate_to_pose')

        self.procs = []
        self.state = 'BOOT'
        self.launch_time = None
        self.undock_done = False    # undock 완료 신호 수신 여부
        self.nav_sent = False
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            'undock_navigator 시작 — 스택 기동 + 도킹 중 localize 후 undock_done 대기')

    def undock_done_callback(self, msg):
        if msg.data and not self.undock_done:
            self.undock_done = True
            self.get_logger().info('undock_done 수신 — ARMED 면 즉시 goal 전송')

    # ── subprocess 런치 ──
    def _popen(self, cmd):
        self.get_logger().info('실행: ' + ' '.join(cmd))
        return subprocess.Popen(cmd)

    def launch(self):
        if self.launch_stacks:
            self.procs.append(self._popen([
                'ros2', 'launch', 'turtlebot4_navigation', 'localization.launch.py',
                f'namespace:={self.prefix}', f'map:={self.map_path}']))
            self.procs.append(self._popen([
                'ros2', 'launch', 'turtlebot4_navigation', 'nav2.launch.py',
                f'namespace:={self.prefix}']))
        if self.launch_rviz:
            self.procs.append(self._popen([
                'ros2', 'launch', 'turtlebot4_viz', 'view_navigation.launch.py',
                f'namespace:={self.prefix}']))

    # ── 상태머신 ──
    def tick(self):
        if self.state == 'BOOT':
            # 로봇이 아직 도킹된 상태에서 스택 기동 (undock 전에!)
            self.launch()
            self.launch_time = self.get_clock().now()
            self.state = 'SETTLE'
            self.get_logger().info(
                f'스택 기동 — {self.startup_delay:.0f}s 동안 도킹 위치에서 AMCL 수렴')
            return

        if self.state == 'SETTLE':
            # 도킹 중 초기 pose 를 반복 발행 → AMCL 이 도크에서 자리잡음
            if self.auto_initial_pose:
                self.publish_initial_pose()
            elapsed = (self.get_clock().now() - self.launch_time).nanoseconds / 1e9
            if elapsed < self.startup_delay:
                return
            if self.nav_client.server_is_ready():
                self.state = 'ARMED'
                self.get_logger().info('ARMED — localize 완료, undock_done 대기 중')
            else:
                self.get_logger().warn(
                    'navigate_to_pose 서버 대기 중... (Nav2 active 확인)',
                    throttle_duration_sec=2.0)
            return

        if self.state == 'ARMED':
            # 도킹 중 localize 완료. undock_done 오면 goal 전송.
            if self.undock_done:
                self.state = 'SEND_GOAL'
            return

        if self.state == 'SEND_GOAL':
            self.send_nav_goal()
            self.state = 'NAVIGATING'
            return

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.goal_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        msg.pose.pose.orientation.z = sin(self.init_yaw / 2.0)
        msg.pose.pose.orientation.w = cos(self.init_yaw / 2.0)
        msg.pose.covariance[0] = 0.25     # x
        msg.pose.covariance[7] = 0.25     # y
        msg.pose.covariance[35] = 0.068   # yaw
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'초기 pose 발행 ({self.init_x}, {self.init_y}, {self.init_yaw})',
            throttle_duration_sec=2.0)

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
            self.get_logger().warn('Navigate goal rejected — 재시도')
            self.nav_sent = False
            self.state = 'SEND_GOAL'
            return

        self.get_logger().info('Navigate goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.get_logger().info(f'Navigate finished: status={future.result().status}')
        self.state = 'DONE'

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
        for p in self.procs:
            p.terminate()   # 노드 종료 시 자식 런치(localization/nav2/rviz)도 종료
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UndockNavigator()
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
