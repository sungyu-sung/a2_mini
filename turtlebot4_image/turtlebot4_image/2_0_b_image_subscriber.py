import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/my_robot2/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Image Subscriber Node Started!')

    def image_callback(self, msg):
        # ROS 이미지 → OpenCV 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # 화면에 표시
        cv2.imshow('Robot2 Camera', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()