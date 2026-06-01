import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np


class CameraAvoidNode(Node):
    def __init__(self):
        super().__init__('camera_avoid_node')

        # ── 파라미터 ──
        self.declare_parameter('namespace', '')
        self.declare_parameter('goal_x', -3.5)          # 목표 지점 (odom 기준)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.25)  # 목표 도달 허용 오차 (m)

        self.declare_parameter('obstacle_min', 0.2)     # 장애물 판단 최소 거리
        self.declare_parameter('obstacle_max', 1.0)     # 장애물 판단 최대 거리 (이보다 멀면 무시)

        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.6)

        ns = self.get_parameter('namespace').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.obs_min = self.get_parameter('obstacle_min').value
        self.obs_max = self.get_parameter('obstacle_max').value
        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value

        prefix = f'/{ns.strip("/")}' if ns.strip('/') else ''
        self.bridge = CvBridge()

        # ── 상태 ──
        self.pose = None          # (x, y, yaw)
        self.obstacle = False     # 카메라에 가까운 장애물 있는지
        self.reached = False

        # ── 통신 ──
        self.create_subscription(Image, f'{prefix}/oakd/rgb/preview/depth',
                                 self.depth_cb, 10)
        self.create_subscription(Odometry, f'{prefix}/odom',
                                 self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)

        # 10Hz 제어 루프
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Camera avoid 시작 (ns: {prefix or "none"})')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}), '
                               f'장애물 판단: {self.obs_min}~{self.obs_max}m')

    # ── depth 콜백: 가까운 장애물 유무만 판단 ──
    def depth_cb(self, msg):
        depth = np.array(self.bridge.imgmsg_to_cv2(msg, 'passthrough'), dtype=np.float32)
        h, w = depth.shape

        # 정면 중앙 영역 (가로 중앙 1/3, 세로 중앙 절반)
        roi = depth[h//4:3*h//4, w//3:2*w//3]
        valid = roi[np.isfinite(roi) & (roi > 0)]

        if len(valid) == 0:
            self.obstacle = False
            return

        # 범위 안(min~max)의 픽셀 수가 충분하면 장애물로 판단 → 멀리있는 벽은 무시
        near = valid[(valid > self.obs_min) & (valid < self.obs_max)]
        self.obstacle = len(near) > (roi.size * 0.05)  # ROI의 5% 이상이면 장애물

    # ── odom 콜백: 현재 위치/방향 ──
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    # ── 제어 루프 ──
    def control_loop(self):
        if self.pose is None or self.reached:
            return

        x, y, yaw = self.pose
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

        twist = Twist()

        # 목표 도달
        if dist < self.goal_tol:
            self.reached = True
            self.pub.publish(Twist())  # 정지
            self.get_logger().info('🎯 목표 도달! 정지')
            return

        if self.obstacle:
            # 장애물 회피: 제자리 회전
            twist.linear.x = 0.0
            twist.angular.z = self.w
            self.get_logger().info('🚧 장애물 회피 중', throttle_duration_sec=1.0)
        else:
            # 목표 방향으로 주행
            target_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(target_yaw - yaw),
                                 math.cos(target_yaw - yaw))

            if abs(yaw_err) > 0.2:
                # 방향 먼저 정렬
                twist.angular.z = self.w if yaw_err > 0 else -self.w
            else:
                # 직진
                twist.linear.x = self.v
                twist.angular.z = 0.5 * yaw_err
            self.get_logger().info(f'➡️  목표까지 {dist:.2f}m',
                                   throttle_duration_sec=1.0)

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CameraAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())  # 종료 시 정지
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
