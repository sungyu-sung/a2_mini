"""yolo_depth_detector — RGB YOLO 탐지 + car bbox 중심 depth 측정 + 정렬·접근 제어 (단일 노드).

선생님 지침대로 RGB·depth 를 **한 파일(한 노드)** 에서 함께 처리한다.

구조(실시간 갱신을 위해):
  - depth 콜백: 가장 최근 depth 프레임을 캐시(self.latest_depth) 만 해 둠
  - RGB 콜백: 매 프레임 YOLO 추론 → bbox 그림 → 'car' 면 "가장 최근 depth" 에서
    bbox 중심 거리를 샘플링 → 화면/토픽 갱신 → (제어 ON) cmd_vel 발행

  (RGB 가 화면 갱신을 주도하므로 창이 실시간으로 계속 갱신됨. depth 는 최근 프레임을
   쓰므로 약간의 시간차는 있으나, 느린 주행에서는 거리 측정에 영향이 거의 없음.
   엄격한 RGB↔depth 시간동기화가 필요하면 message_filters 로 바꿀 수 있음.)

기능:
  1. RGB 카메라로 YOLO 객체 탐지 (클래스: car, dummy)
  2. 'car' 탐지 시 그 bbox 중심의 depth(거리) 를 stereo depth 로 측정
  3. (enable_control=True) car 의 bbox 가 화면 중앙에 오도록 조향하고,
     target_distance_m(기본 1.0 m) 까지 전진해 정지.
     - car 가 안 보이면 제자리 회전하며 탐색(SEARCHING)
     - car 가 보이면 중심 정렬 후 거리 1 m 까지 접근(APPROACHING) → 도달 시 정지(DONE)

제어 상태머신(매 RGB 프레임마다 갱신):
  SEARCHING ──(car 발견)──> APPROACHING ──(중앙 정렬 & 거리<=1m)──> DONE
     ▲                          │
     └────────(car 분실)─────────┘   (DONE 에서도 car 분실 시 SEARCHING 복귀)

참고: turtlebot4_yolo.yolo_detector(YOLO) + rokey_pjt.depth_checker(depth mm→m).
QoS: 카메라/depth 는 sensor QoS(BEST_EFFORT) 로 구독(둘 다 수신 호환).

한계(TODO): RGB 와 stereo depth 는 해상도·FOV 가 달라, 여기서는 해상도 비율로 중심 픽셀을
매핑한다. FOV/정렬 차가 크면 오차가 생기므로, 정밀 측정이 필요하면 depth-to-RGB 정렬을 적용할 것.
"""
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage

from ultralytics import YOLO


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class YoloDepthDetector(Node):
    def __init__(self):
        super().__init__('yolo_depth_detector')

        # ---- parameters ----
        self.declare_parameter('model_path', '')   # 비우면 패키지 share의 models/robotview_best.pt
        # compressed 사용 시 full 해상도라도 JPEG(~50KB)라 wifi 지연이 작음 → 기본 full + compressed
        self.declare_parameter('rgb_topic', '/robot2/oakd/rgb/image_raw')
        self.declare_parameter('use_compressed', True)   # True면 <rgb_topic>/compressed 구독·디코드
        self.declare_parameter('depth_topic', '/robot2/oakd/stereo/image_raw')
        self.declare_parameter('annotated_topic', '/yolo_depth/image_annotated')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('target_class', 'car')   # 거리 측정 대상 클래스
        self.declare_parameter('show', True)            # cv2 창으로 표시 여부

        # ---- 정렬·접근 제어 파라미터 ----
        self.declare_parameter('enable_control', True)        # False면 탐지만(주행 안 함)
        self.declare_parameter('cmd_vel_topic', '/robot2/cmd_vel')
        self.declare_parameter('target_distance_m', 1.0)      # 이 거리까지 접근 후 정지
        self.declare_parameter('dist_tol_m', 0.1)             # 목표거리 허용오차(정지 밴드)
        self.declare_parameter('center_tol', 0.08)            # |수평오차| 이하면 '정렬됨'
        self.declare_parameter('align_gate', 0.20)            # |수평오차| 이하라야 전진 허용
        self.declare_parameter('yaw_kp', 0.8)                 # 수평오차(정규화)→angular P 게인
        self.declare_parameter('max_angular', 0.6)            # rad/s 상한
        self.declare_parameter('approach_kp', 0.5)            # 거리오차(m)→linear P 게인
        self.declare_parameter('max_linear', 0.15)            # m/s 상한
        self.declare_parameter('search_angular', 0.4)         # 탐색 회전 속도 rad/s

        model_path = self.get_parameter('model_path').value
        rgb_topic = self.get_parameter('rgb_topic').value
        use_compressed = bool(self.get_parameter('use_compressed').value)
        depth_topic = self.get_parameter('depth_topic').value
        self.conf = float(self.get_parameter('conf').value)
        self.target_class = self.get_parameter('target_class').value
        self.show = bool(self.get_parameter('show').value)

        self.enable_control = bool(self.get_parameter('enable_control').value)
        self.target_distance = float(self.get_parameter('target_distance_m').value)
        self.dist_tol = float(self.get_parameter('dist_tol_m').value)
        self.center_tol = float(self.get_parameter('center_tol').value)
        self.align_gate = float(self.get_parameter('align_gate').value)
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.approach_kp = float(self.get_parameter('approach_kp').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.search_angular = float(self.get_parameter('search_angular').value)
        self.state = 'SEARCHING'   # SEARCHING / APPROACHING / DONE

        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('car_mission'), 'models', 'robotview_best.pt')
        if not os.path.exists(model_path):
            self.get_logger().error(f'모델 없음: {model_path}')
            raise FileNotFoundError(model_path)
        self.get_logger().info(f'YOLO 로드: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'클래스: {self.model.names}')

        self.bridge = CvBridge()
        self.latest_depth = None    # 가장 최근 depth 프레임 (mm)
        self.display_frame = None   # 화면에 그릴 최신 annotated 프레임 (main 루프가 표시)
        self.frame_count = 0

        self.pub = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 10)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # 항상 "최신 프레임만" 받도록 BEST_EFFORT + KEEP_LAST depth=1
        # (depth를 키우거나 RELIABLE이면 느린 wifi에서 오래된 프레임이 쌓여 지연 누적 → 과거 영상 배치처럼 보임)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        # depth: 최근 프레임만 캐시 / RGB: 화면 갱신 주도
        self.create_subscription(Image, depth_topic, self.on_depth, qos)
        if use_compressed:
            rgb_in = rgb_topic + '/compressed'
            self.create_subscription(CompressedImage, rgb_in, self.on_rgb_compressed, qos)
        else:
            rgb_in = rgb_topic
            self.create_subscription(Image, rgb_in, self.on_rgb, qos)

        ctrl = (f'ON → {self.get_parameter("cmd_vel_topic").value} '
                f'(목표 {self.target_distance:.2f} m)') if self.enable_control else 'OFF(탐지만)'
        self.get_logger().info(
            f'yolo_depth_detector 시작 — RGB:{rgb_in} + DEPTH:{depth_topic} | 제어:{ctrl}')

    def on_depth(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def on_rgb(self, msg):
        self._process(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def on_rgb_compressed(self, msg):
        self._process(self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8'))

    @staticmethod
    def _sample_depth_mm(depth, u, v, win=2):
        """(u,v) 주변 (2*win+1)^2 패치에서 0이 아닌 depth 의 중앙값(mm). 없으면 NaN."""
        h, w = depth.shape[:2]
        u0, u1 = max(0, u - win), min(w, u + win + 1)
        v0, v1 = max(0, v - win), min(h, v + win + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32)
        vals = patch[patch > 0]
        if vals.size == 0:
            return float('nan')
        return float(np.median(vals))

    def _process(self, rgb):
        rh, rw = rgb.shape[:2]

        result = self.model.predict(rgb, conf=self.conf, verbose=False)[0]
        annotated = result.plot()

        depth = self.latest_depth   # 가장 최근 depth 프레임
        best = None                 # 제어 대상: 가장 큰 car bbox (가장 가깝/뚜렷)
        if result.boxes is not None:
            for b in result.boxes:
                name = self.model.names[int(b.cls[0])]
                if name != self.target_class:
                    continue  # 거리 측정은 target_class(car)만

                x1, y1, x2, y2 = b.xyxy[0].tolist()
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

                dist_m = float('nan')
                if depth is None:
                    label = 'depth 대기'
                else:
                    dh, dw = depth.shape[:2]
                    du = min(dw - 1, max(0, int(cx * dw / rw)))
                    dv = min(dh - 1, max(0, int(cy * dh / rh)))
                    dist_mm = self._sample_depth_mm(depth, du, dv)
                    if dist_mm == dist_mm:
                        dist_m = dist_mm / 1000.0
                    label = f'{dist_m:.2f} m' if dist_m == dist_m else 'no depth'

                cv2.putText(annotated, label, (int(x1), max(0, int(y1) - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                area = (x2 - x1) * (y2 - y1)
                if best is None or area > best['area']:
                    best = {'area': area, 'cx': cx, 'dist_m': dist_m}

        # ---- 정렬·접근 제어 ----
        if self.enable_control:
            self._control(best, rw, annotated)

        self.pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        self.display_frame = annotated   # 표시는 main 루프가 담당 (GUI 갱신 안정화)

        # 살아있음 표시 (30프레임마다)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'{self.frame_count} 프레임 처리 (depth {"수신중" if depth is not None else "대기"})')

    def _control(self, best, rw, annotated):
        """car bbox 를 화면 중앙에 정렬하고 target_distance 까지 접근.

        best: {'cx','dist_m'} 또는 None(car 미검출). rw: RGB 폭(px). annotated: HUD overlay 대상.
        """
        cmd = Twist()
        hud = ''

        if best is None:
            # SEARCHING — car 가 안 보이면 제자리 회전하며 탐색
            self.state = 'SEARCHING'
            cmd.angular.z = self.search_angular
            hud = 'SEARCHING (car 탐색중)'
        else:
            # 수평 오차: +면 car 가 화면 오른쪽 → 우회전(angular.z 음수, ROS 양수=좌회전)
            err_x = (best['cx'] - rw / 2.0) / (rw / 2.0)   # -1..+1
            cmd.angular.z = _clamp(-self.yaw_kp * err_x, -self.max_angular, self.max_angular)
            aligned = abs(err_x) < self.center_tol
            dist = best['dist_m']

            if dist != dist:  # depth 없음 → 정렬만(전진 안 함)
                self.state = 'APPROACHING'
                hud = f'APPROACHING err={err_x:+.2f} dist=no depth (정렬만)'
            elif dist <= self.target_distance + self.dist_tol:
                # 목표 거리 도달 → 정지. 정렬까지 되면 DONE.
                cmd.linear.x = 0.0
                if aligned:
                    cmd.angular.z = 0.0
                    self.state = 'DONE'
                    hud = f'DONE  dist={dist:.2f} m (정지)'
                else:
                    self.state = 'APPROACHING'
                    hud = f'ALIGNING err={err_x:+.2f} dist={dist:.2f} m'
            else:
                # 접근 — 충분히 정렬됐을 때만 전진(아니면 회전 우선)
                self.state = 'APPROACHING'
                if abs(err_x) < self.align_gate:
                    cmd.linear.x = _clamp(
                        self.approach_kp * (dist - self.target_distance), 0.0, self.max_linear)
                hud = (f'APPROACHING err={err_x:+.2f} dist={dist:.2f} m '
                       f'v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f}')

        self.cmd_pub.publish(cmd)
        cv2.putText(annotated, hud, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def _stop(self):
        if self.enable_control:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthDetector()
    # imshow/waitKey 는 메인 루프에서 (rclpy.spin() + 콜백 내 imshow 는 창이 안 갱신되는 문제 → spin_once 루프 사용)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.show and node.display_frame is not None:
                cv2.imshow('YOLO + Depth', node.display_frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()   # 종료 시 로봇 정지(cmd_vel 0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
