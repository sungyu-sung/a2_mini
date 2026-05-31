import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        # Subscribe to republished topics
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/my_robot2/battery_state',
            self.battery_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/my_robot2/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info('Data Subscriber Node Started!')

    def battery_callback(self, msg):
        self.get_logger().info(f'[Battery] {msg.percentage * 100:.1f}% | voltage: {msg.voltage:.2f}V')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'[Odom] x: {x:.2f}, y: {y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()