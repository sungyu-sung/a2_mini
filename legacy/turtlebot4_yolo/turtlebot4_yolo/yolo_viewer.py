import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class YoloViewer(Node):
    """bounding box가 그려진 이미지를 구독해 화면에 표시하는 노드.
    (turtlebot4_image의 image_subscriber와 동일한 역할)"""

    def __init__(self):
        super().__init__('yolo_viewer')

        self.declare_parameter('annotated_topic', '/yolo/image_annotated')
        annotated_topic = self.get_parameter('annotated_topic').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, annotated_topic, self.image_callback, 10
        )
        self.get_logger().info(f'YOLO Viewer 시작! 구독: {annotated_topic}')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('YOLO Detection', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
