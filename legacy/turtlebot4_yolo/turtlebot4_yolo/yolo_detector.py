import json
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ultralytics import YOLO


class YoloDetector(Node):
    """카메라 이미지를 구독해 best.pt(YOLO)로 추론하고,
    bounding box가 그려진 이미지와 검출 결과를 발행하는 노드."""

    def __init__(self):
        super().__init__('yolo_detector')

        # ---- parameters (best.pt만 넣으면 동작하도록 기본값 제공) ----
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_topic', '/robot2/oakd/rgb/preview/image_raw')
        self.declare_parameter('annotated_topic', '/yolo/image_annotated')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('device', '')  # '' = 자동(가능하면 GPU), 'cpu', '0' 등
        self.declare_parameter('publish_detections', True)

        model_path = self.get_parameter('model_path').value
        input_topic = self.get_parameter('input_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.conf = float(self.get_parameter('conf').value)
        device = self.get_parameter('device').value
        self.device = device if device else None
        self.publish_detections = bool(self.get_parameter('publish_detections').value)

        # model_path가 비어 있으면 패키지 share의 models/robotview_best.pt 사용
        if not model_path:
            share_dir = get_package_share_directory('turtlebot4_yolo')
            model_path = os.path.join(share_dir, 'models', 'robotview_best.pt')

        if not os.path.exists(model_path):
            self.get_logger().error(
                f"모델 파일을 찾을 수 없습니다: {model_path}\n"
                "robotview_best.pt를 turtlebot4_yolo/models/ 에 넣고 다시 빌드하거나, "
                "model_path 파라미터로 절대 경로를 지정하세요."
            )
            raise FileNotFoundError(model_path)

        self.get_logger().info(f'YOLO 모델 로드 중: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'클래스: {self.model.names}')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, 10
        )
        self.image_pub = self.create_publisher(Image, annotated_topic, 10)
        if self.publish_detections:
            self.det_pub = self.create_publisher(String, detections_topic, 10)

        self.get_logger().info(
            f"YOLO Detector 시작! 구독: {input_topic} → 발행: {annotated_topic}"
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return

        # 추론
        results = self.model.predict(
            frame, conf=self.conf, device=self.device, verbose=False
        )
        result = results[0]

        # bounding box가 그려진 이미지
        annotated = result.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        annotated_msg.header = msg.header
        self.image_pub.publish(annotated_msg)

        # 검출 결과 발행 (JSON 문자열)
        detections = []
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                cls_id = int(box.cls[0])
                detections.append({
                    'class_id': cls_id,
                    'class_name': self.model.names.get(cls_id, str(cls_id)),
                    'conf': round(float(box.conf[0]), 3),
                    'bbox_xyxy': [round(float(v), 1) for v in box.xyxy[0].tolist()],
                })

        if self.publish_detections:
            det_msg = String()
            det_msg.data = json.dumps({
                'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'detections': detections,
            })
            self.det_pub.publish(det_msg)

        self.get_logger().info(f'검출 {len(detections)}개')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
