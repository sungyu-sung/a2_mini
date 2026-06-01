import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class MoveP1Node(Node):
    def __init__(self):
        super().__init__('move_p1_node')

        # ── 파라미터 ──
        self.declare_parameter('namespace', '')      # 실로봇: robot2
        self.declare_parameter('goal_x', 1.0)        # 목표 좌표 (map 기준)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_yaw', 0.0)      # 목표 방향 (rad)
        self.declare_parameter('start_delay', 8.0)   # nav2 준비 대기 시간 (s)

        ns = self.get_parameter('namespace').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        delay = self.get_parameter('start_delay').value

        prefix = f'/{ns.strip("/")}' if ns.strip('/') else ''

        # ── Nav2 액션 클라이언트 ──
        self.nav_client = ActionClient(
            self, NavigateToPose, f'{prefix}/navigate_to_pose')

        self.get_logger().info('move_p1 시작 (노드 실행 시 바로 목표로 이동)')
        self.get_logger().info(f'목표 좌표: ({self.goal_x}, {self.goal_y}), yaw={self.goal_yaw}')
        self.get_logger().info(f'Nav2 액션: {prefix}/navigate_to_pose')
        self.get_logger().info(f'{delay}초 후 출발...')

        # nav2 준비될 시간을 준 뒤 한 번만 목표 전송
        self._timer = self.create_timer(delay, self._start_once)

    def _start_once(self):
        self._timer.cancel()  # 1회만 실행
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Nav2 액션 서버 대기 중...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 액션 서버 없음 (nav2 실행됐는지 확인)')
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        goal.pose = pose

        self.get_logger().info(f'🚀 목표 전송: ({self.goal_x}, {self.goal_y})')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('목표 거부됨 (nav2 아직 준비 안 됨) → 3초 후 재시도')
            self._retry_timer = self.create_timer(3.0, self._retry_once)
            return
        self.get_logger().info('목표 수락됨 → 이동 중')
        handle.get_result_async().add_done_callback(self.result_cb)

    def _retry_once(self):
        self._retry_timer.cancel()
        self.send_goal()

    def result_cb(self, future):
        self.get_logger().info('🎯 목표 도달 완료')


def main(args=None):
    rclpy.init(args=args)
    node = MoveP1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
