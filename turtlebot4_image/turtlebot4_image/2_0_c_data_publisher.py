import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        # Subscribe to robot2 battery and odom topics
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/robot2/battery_state',
            self.battery_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odom',
            self.odom_callback,
            10
        )
        # Republish to new topics
        self.battery_pub = self.create_publisher(BatteryState, '/my_robot2/battery_state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/my_robot2/odom', 10)
        self.get_logger().info('Data Publisher Node Started!')

    def battery_callback(self, msg):
        self.battery_pub.publish(msg)
        self.get_logger().info(f'Battery: {msg.percentage * 100:.1f}%')

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'Position - x: {x:.2f}, y: {y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()