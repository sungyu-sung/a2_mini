"""car_tracker — 로봇캠 YOLO 검출 + depth 융합 → 차량 방위/거리 발행 (PLACEHOLDER).

입력:
  - /yolo/detections (std_msgs/String, JSON)  ← turtlebot4_yolo.yolo_detector (로봇 OAK-D RGB)
  - /robot2/oakd/stereo/image_raw (sensor_msgs/Image)  ← depth (거리 산출용)
출력 (mission_manager 의 SEARCH/APPROACH 가 사용):
  - /car/visible  (Bool)
  - /car/bearing  (Float32)  정규화 수평 오프셋 (- 왼쪽 / + 오른쪽, 화면중앙=0)
  - /car/distance (Float32)  m, 모르면 NaN

TODO:
  - depth 영상에서 bbox 중심의 거리 샘플링 (rokey_pjt.depth_checker 로직 재사용).
  - RGB(검출)과 depth 의 해상도/시야 정렬 보정.
  - bearing 계산용 이미지 width 확보 (현재는 image_width 파라미터로 가정).
"""
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class CarTracker(Node):
    def __init__(self):
        super().__init__('car_tracker')

        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('depth_topic', '/robot2/oakd/stereo/image_raw')
        self.declare_parameter('car_class_name', 'car')
        self.declare_parameter('image_width', 320)   # 로봇 RGB preview 기본 폭 (TODO: 실측)

        self.car_class = self.get_parameter('car_class_name').value
        self.image_width = int(self.get_parameter('image_width').value)

        self.create_subscription(
            String, self.get_parameter('detections_topic').value, self._on_det, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, 10)

        self.visible_pub = self.create_publisher(Bool, '/car/visible', 10)
        self.bearing_pub = self.create_publisher(Float32, '/car/bearing', 10)
        self.distance_pub = self.create_publisher(Float32, '/car/distance', 10)

        self._last_bbox = None  # (cx, cy) 최근 차량 중심 (depth 샘플링용)
        self.get_logger().info('car_tracker 시작 (PLACEHOLDER)')

    def _on_det(self, msg):
        try:
            dets = json.loads(msg.data).get('detections', [])
        except json.JSONDecodeError:
            return

        cars = [d for d in dets if d.get('class_name') == self.car_class]
        if not cars:
            self._last_bbox = None
            self.visible_pub.publish(Bool(data=False))
            self.distance_pub.publish(Float32(data=float('nan')))
            return

        # conf 최고 차량 선택
        car = max(cars, key=lambda d: d.get('conf', 0.0))
        x1, y1, x2, y2 = car['bbox_xyxy']
        cx = (x1 + x2) / 2.0
        self._last_bbox = (cx, (y1 + y2) / 2.0)

        bearing = (cx - self.image_width / 2.0) / (self.image_width / 2.0)
        self.visible_pub.publish(Bool(data=True))
        self.bearing_pub.publish(Float32(data=float(bearing)))
        # 거리는 depth 콜백에서 채움 (없으면 NaN 유지)

    def _on_depth(self, msg):
        # TODO: self._last_bbox 위치의 depth 픽셀값을 읽어 거리(m) 산출 후 발행.
        #   rokey_pjt.depth_checker 의 픽셀→거리 변환 로직 재사용.
        if self._last_bbox is None:
            return
        # distance = sample_depth(msg, self._last_bbox)
        # self.distance_pub.publish(Float32(data=distance))
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CarTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
