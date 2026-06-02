"""
YOLO 기반 추적 노드 (실로봇용).

기존 색검출(tracking_node.py)과 추적/거리 로직은 동일하고,
'대상 검출' 부분만 HSV → YOLO로 교체한 버전.

⚠️ 아직 .pt 모델이 없어 detection 부분은 뼈대만 있음.
   모델을 받으면:
     1. model_path 파라미터에 .pt 경로 지정
     2. detect() 안의 YOLO 추론 주석을 해제
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
import math


class TrackingYoloNode(Node):
    def __init__(self):
        super().__init__('tracking_yolo_node')

        # ── 파라미터 ──
        self.declare_parameter('namespace', '')
        self.declare_parameter('model_path', '')         # YOLO .pt 경로 (받으면 채움)
        self.declare_parameter('target_class', '')       # 추적할 클래스 이름
        self.declare_parameter('conf', 0.5)              # 검출 신뢰도 임계값
        self.declare_parameter('target_distance', 0.3)   # 0.3m까지 붙고, 더 가까우면 후진
        self.declare_parameter('distance_tol', 0.05)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('center_tol', 30)
        self.declare_parameter('enabled', True)
        self.declare_parameter('search_speed', 0.5)   # 차 놓쳤을 때 탐색 회전 속도(rad/s)
        # 토픽 (기본값: 실물 OAK-D compressed = 빠름). 시뮬은 토픽 덮어쓰기
        self.declare_parameter('rgb_topic', '/oakd/rgb/image_raw/compressed')
        self.declare_parameter('depth_topic', '/oakd/stereo/image_raw')
        self.declare_parameter('use_compressed', True)   # RGB를 compressed로 받기
        # depth 단위 → m 변환 계수 (실물 mm=0.001, 시뮬 m=1.0)
        self.declare_parameter('depth_scale', 0.001)

        ns = self.get_parameter('namespace').value
        self.model_path = self.get_parameter('model_path').value
        self.target_class = self.get_parameter('target_class').value
        self.conf = self.get_parameter('conf').value
        self.enabled = self.get_parameter('enabled').value
        self.target_dist = self.get_parameter('target_distance').value
        self.dist_tol = self.get_parameter('distance_tol').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.center_tol = self.get_parameter('center_tol').value
        self.search_speed = self.get_parameter('search_speed').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.depth_scale = self.get_parameter('depth_scale').value

        prefix = f'/{ns.strip("/")}' if ns.strip('/') else ''
        self.bridge = CvBridge()
        self.depth = None
        self.target_px = None      # 검출된 대상 중심 픽셀 (u, v)
        self.img_w = None
        self.img_h = None

        # ── 탐색 상태 ──
        self.search_ticks = 0      # 차 놓친 뒤 탐색 회전 누적 틱(0.1s 단위)
        # 한 바퀴(2π) 도는 데 필요한 틱 수
        self.search_limit = int((2 * math.pi / self.search_speed) / 0.1)

        # ── YOLO 모델 로드 (모델 경로 있을 때만) ──
        self.model = None
        if self.model_path:
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLO 모델 로드: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'YOLO 로드 실패: {e}')
        else:
            self.get_logger().warn('model_path 미지정 — detection 비활성 (뼈대 모드)')

        if self.use_compressed:
            self.create_subscription(CompressedImage, f'{prefix}{rgb_topic}',
                                     self.rgb_compressed_cb, 10)
        else:
            self.create_subscription(Image, f'{prefix}{rgb_topic}',
                                     self.rgb_cb, 10)
        self.create_subscription(Image, f'{prefix}{depth_topic}',
                                 self.depth_cb, 10)
        self.create_subscription(Bool, f'{prefix}/tracking_enable',
                                 self.enable_cb, 10)
        self.pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'tracking_yolo 시작 (ns: {prefix or "none"})')

    # ── 대상 검출: YOLO 추론 → 중심 픽셀 ──
    def detect(self, img):
        if self.model is None:
            return None
        results = self.model(img, conf=self.conf, verbose=False)
        best = None
        best_area = 0
        for r in results:
            for box in r.boxes:
                cls_name = self.model.names[int(box.cls[0])]
                if self.target_class and cls_name != self.target_class:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area = area
                    best = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        return best

    def rgb_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.img_h, self.img_w = img.shape[0], img.shape[1]
        self.target_px = self.detect(img)

    def rgb_compressed_cb(self, msg):
        # 압축 이미지 디코드 (실시간성 ↑)
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        self.img_h, self.img_w = img.shape[0], img.shape[1]
        self.target_px = self.detect(img)

    def depth_cb(self, msg):
        self.depth = np.array(self.bridge.imgmsg_to_cv2(msg, 'passthrough'),
                              dtype=np.float32)

    def enable_cb(self, msg):
        self.enabled = msg.data
        self.get_logger().info(f'추적 {"활성화" if self.enabled else "비활성화"}')
        if not self.enabled:
            self.pub.publish(Twist())

    def control_loop(self):
        if not self.enabled:
            return

        twist = Twist()

        # ── 차를 놓친 경우 ──
        if self.target_px is None or self.img_w is None:
            if self.search_ticks >= self.search_limit:
                # 한 바퀴 다 돌아도 못 찾음 → 가만히 대기 (차 다시 보이면 재개)
                self.pub.publish(Twist())
                self.get_logger().info('⏸  차 못 찾음 — 대기 중 (다시 보이면 추격)',
                                       throttle_duration_sec=3.0)
                return
            # 제자리 회전하며 탐색
            self.search_ticks += 1
            twist.angular.z = self.search_speed
            self.get_logger().info('🔄 차 놓침 — 탐색 회전 중',
                                   throttle_duration_sec=1.0)
            self.pub.publish(twist)
            return

        # ── 차를 찾음: 탐색 카운터 리셋 (다시 추격 시작) ──
        self.search_ticks = 0

        u, v = self.target_px
        cx = self.img_w // 2
        err_x = u - cx

        # 중앙 정렬 회전 (항상)
        if abs(err_x) > self.center_tol:
            ang = -float(err_x) / cx * self.max_ang
            twist.angular.z = max(-self.max_ang, min(self.max_ang, ang))

        # 거리: 0.3m보다 멀면 전진(붙음). 가까우면 전진 안 함(중앙정렬만, 후진 없음)
        if self.depth is not None and self.img_h is not None:
            dh, dw = self.depth.shape
            du = max(0, min(dw - 1, int(u * dw / self.img_w)))
            dv = max(0, min(dh - 1, int(v * dh / self.img_h)))
            patch = self.depth[max(0, dv-10):dv+10, max(0, du-10):du+10]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if len(valid) > 0:
                dist = float(np.median(valid)) * self.depth_scale
                if dist > self.target_dist + self.dist_tol:
                    twist.linear.x = min(self.max_lin, (dist - self.target_dist) * 0.5)
                # 0.3m 이내면 전진/후진 없이 중앙 정렬만
                self.get_logger().info(f'대상 거리 {dist:.2f}m, 중심오차 {err_x}px',
                                       throttle_duration_sec=0.5)

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
