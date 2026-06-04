import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # Subscribe to robot2 camera topic
        self.subscription = self.create_subscription(
            Image,
            '/robot2/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        # Republish to new topic
        self.publisher = self.create_publisher(
            Image,
            '/my_robot2/image_raw',
            10
        )
        self.get_logger().info('Image Publisher Node Started!')

    def image_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info('Publishing image...')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()