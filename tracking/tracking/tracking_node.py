import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')

        # ── 파라미터 ──
        self.declare_parameter('namespace', '')
        self.declare_parameter('target_distance', 0.8)   # 유지할 거리 (m)
        self.declare_parameter('distance_tol', 0.15)     # 거리 허용 오차
        self.declare_parameter('max_linear', 0.25)       # 최대 전진 속도
        self.declare_parameter('max_angular', 0.8)       # 최대 회전 속도
        self.declare_parameter('center_tol', 30)         # 중심 허용 픽셀 오차

        ns = self.get_parameter('namespace').value
        self.target_dist = self.get_parameter('target_distance').value
        self.dist_tol = self.get_parameter('distance_tol').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.center_tol = self.get_parameter('center_tol').value

        prefix = f'/{ns.strip("/")}' if ns.strip('/') else ''

        self.bridge = CvBridge()
        self.depth = None          # 최근 depth 이미지
        self.cube_px = None        # 큐브 중심 픽셀 (u, v)
        self.img_w = None
        self.img_h = None

        self.create_subscription(Image, f'{prefix}/oakd/rgb/preview/image_raw',
                                 self.rgb_cb, 10)
        self.create_subscription(Image, f'{prefix}/oakd/rgb/preview/depth',
                                 self.depth_cb, 10)
        self.pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)  # 10Hz 제어

        self.get_logger().info(f'tracking 시작 (ns: {prefix or "none"})')
        self.get_logger().info(f'유지 거리: {self.target_dist}m')

    # 파란색 검출 → 중심 픽셀
    def rgb_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.img_h, self.img_w = img.shape[0], img.shape[1]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 파란색 HSV 범위
        lower = np.array([100, 120, 50])
        upper = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        # 가장 큰 파란 덩어리 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.cube_px = None
            return
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 100:   # 너무 작으면 무시
            self.cube_px = None
            return
        M = cv2.moments(c)
        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])
        self.cube_px = (u, v)

    def depth_cb(self, msg):
        self.depth = np.array(self.bridge.imgmsg_to_cv2(msg, 'passthrough'),
                              dtype=np.float32)

    def control_loop(self):
        twist = Twist()

        # 큐브 안 보이면 정지
        if self.cube_px is None or self.img_w is None:
            self.pub.publish(twist)
            return

        u, v = self.cube_px
        cx = self.img_w // 2
        err_x = u - cx   # 양수=큐브가 오른쪽 → 우회전 필요

        # ── 회전: 큐브를 중심에 ──
        if abs(err_x) > self.center_tol:
            # 정규화 비례 제어
            ang = -float(err_x) / cx * self.max_ang
            twist.angular.z = max(-self.max_ang, min(self.max_ang, ang))

        # ── 전진/후진: 거리 유지 ──
        if self.depth is not None and self.img_h is not None:
            dh, dw = self.depth.shape
            # RGB 픽셀 (u,v) → depth 이미지 픽셀로 비율 변환
            du = int(u * dw / self.img_w)
            dv = int(v * dh / self.img_h)
            du = max(0, min(dw - 1, du))
            dv = max(0, min(dh - 1, dv))
            # 큐브 중심 주변 작은 영역의 거리 중앙값
            patch = self.depth[max(0, dv-10):dv+10, max(0, du-10):du+10]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if len(valid) > 0:
                dist = float(np.median(valid))
                if dist > self.target_dist + self.dist_tol:
                    twist.linear.x = min(self.max_lin,
                                         (dist - self.target_dist) * 0.5)
                elif dist < self.target_dist - self.dist_tol:
                    twist.linear.x = max(-self.max_lin,
                                         (dist - self.target_dist) * 0.5)
                self.get_logger().info(
                    f'큐브 거리 {dist:.2f}m, 중심오차 {err_x}px',
                    throttle_duration_sec=0.5)

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
