import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class SimMainNode(Node):
    """
    전체 시나리오 지휘 (오케스트레이터).
      IDLE  → (외부 트리거) → NAVIGATING → (P1 도착) → TRACKING
    - 이동은 Nav2 navigate_to_pose 액션
    - 추적은 tracking 노드에 /tracking_enable True 발행
    """

    def __init__(self):
        super().__init__('main_sim_node')

        # ── 파라미터 ──
        self.declare_parameter('namespace', '')
        self.declare_parameter('trigger_topic', '/cctv_trigger')
        self.declare_parameter('p1_x', -2.93)     # P1 좌표 (map 기준)
        self.declare_parameter('p1_y', 6.91)
        self.declare_parameter('p1_yaw', 4.10)    # 도착 시 바라볼 방향 (큐브 쪽)

        ns = self.get_parameter('namespace').value
        trig = self.get_parameter('trigger_topic').value
        self.p1_x = self.get_parameter('p1_x').value
        self.p1_y = self.get_parameter('p1_y').value
        self.p1_yaw = self.get_parameter('p1_yaw').value

        prefix = f'/{ns.strip("/")}' if ns.strip('/') else ''

        # ── 통신 ──
        self.nav_client = ActionClient(self, NavigateToPose,
                                       f'{prefix}/navigate_to_pose')
        self.track_pub = self.create_publisher(Bool, f'{prefix}/tracking_enable', 10)
        self.create_subscription(Bool, trig, self.trigger_cb, 10)

        self.state = 'IDLE'

        # 시작 시 추적은 꺼둠
        self._publish_track(False)

        self.get_logger().info('main_sim 시작 — 외부 트리거 대기 중 (IDLE)')
        self.get_logger().info(f'트리거 토픽: {trig}')
        self.get_logger().info(f'P1 목표: ({self.p1_x}, {self.p1_y}), yaw={self.p1_yaw}')

    def _publish_track(self, on):
        msg = Bool()
        msg.data = on
        self.track_pub.publish(msg)

    # ── 외부 트리거 ──
    def trigger_cb(self, msg):
        if not msg.data:
            return
        if self.state != 'IDLE':
            self.get_logger().warn(f'트리거 무시 (현재 상태: {self.state})')
            return
        self.get_logger().info('📨 외부 신호 수신 → P1으로 이동 시작')
        self._publish_track(False)   # 혹시 켜져 있으면 끄고
        self.state = 'NAVIGATING'
        self.send_nav_goal()

    # ── P1 이동 ──
    def send_nav_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 액션 서버 없음 → IDLE 복귀')
            self.state = 'IDLE'
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.p1_x
        pose.pose.position.y = self.p1_y
        pose.pose.orientation.z = math.sin(self.p1_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.p1_yaw / 2.0)
        goal.pose = pose

        self.get_logger().info(f'🚀 P1 이동: ({self.p1_x}, {self.p1_y})')
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self.nav_response_cb)

    def nav_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('P1 목표 거부됨 → 3초 후 재시도')
            self._retry = self.create_timer(3.0, self._retry_nav)
            return
        self.get_logger().info('P1 목표 수락됨 → 이동 중')
        handle.get_result_async().add_done_callback(self.nav_result_cb)

    def _retry_nav(self):
        self._retry.cancel()
        self.send_nav_goal()

    def nav_result_cb(self, future):
        self.get_logger().info('🎯 P1 도착 → 추적 시작 (TRACKING)')
        self.state = 'TRACKING'
        self._publish_track(True)    # tracking 노드 켜기


def main(args=None):
    rclpy.init(args=args)
    node = SimMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
