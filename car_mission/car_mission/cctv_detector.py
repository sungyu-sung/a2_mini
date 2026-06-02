"""cctv_detector — PC에 USB로 연결된 CCTV 카메라 + YOLO 로 'car' 감지 (PLACEHOLDER).

로봇 카메라가 아니라 **PC에 케이블로 연결된 고정 카메라**(cv2.VideoCapture)를 사용합니다.
'car' 가 감지되면 /cctv/car_detected (Bool=True) 를 발행해 mission_manager 를 출동시킵니다.

모델은 기본적으로 turtlebot4_yolo 패키지의 models/best.pt 를 재사용합니다.
TODO: 오탐 방지를 위한 디바운스(N프레임 연속 감지), 카메라 해상도/index 실측.
"""
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ultralytics import YOLO


class CctvDetector(Node):
    def __init__(self):
        super().__init__('cctv_detector')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', '')      # 비우면 turtlebot4_yolo/models/best.pt
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('car_class_name', 'car')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('detected_topic', '/cctv/car_detected')
        self.declare_parameter('annotated_topic', '/cctv/image_annotated')
        self.declare_parameter('detect_frames', 3)    # N프레임 연속 감지 시 True (디바운스)

        cam_index = int(self.get_parameter('camera_index').value)
        model_path = self.get_parameter('model_path').value
        self.conf = float(self.get_parameter('conf').value)
        self.car_class = self.get_parameter('car_class_name').value
        fps = float(self.get_parameter('fps').value)
        self.detect_frames = int(self.get_parameter('detect_frames').value)

        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('turtlebot4_yolo'), 'models', 'best.pt')
        if not os.path.exists(model_path):
            self.get_logger().error(f'모델 없음: {model_path}')
            raise FileNotFoundError(model_path)
        self.model = YOLO(model_path)

        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'CCTV 카메라 열기 실패 (index={cam_index})')
            raise RuntimeError('camera open failed')

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(
            Bool, self.get_parameter('detected_topic').value, 10)
        self.img_pub = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 10)

        self._hit_count = 0
        self.create_timer(1.0 / max(fps, 1.0), self._tick)
        self.get_logger().info(f'cctv_detector 시작 (camera_index={cam_index})')

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('프레임 읽기 실패')
            return

        result = self.model.predict(frame, conf=self.conf, verbose=False)[0]
        names = self.model.names
        found = any(names[int(b.cls[0])] == self.car_class for b in result.boxes) \
            if result.boxes is not None else False

        # 디바운스: detect_frames 연속 감지 시에만 True
        self._hit_count = self._hit_count + 1 if found else 0
        detected = self._hit_count >= self.detect_frames

        self.det_pub.publish(Bool(data=detected))
        annotated = self.bridge.cv2_to_imgmsg(result.plot(), 'bgr8')
        self.img_pub.publish(annotated)

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CctvDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
