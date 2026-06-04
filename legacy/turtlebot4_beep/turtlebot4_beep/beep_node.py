import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.publisher = self.create_publisher(
            AudioNoteVector,
            '/robot2/cmd_audio',
            10
        )
        # 1초마다 삐뽀삐뽀 실행
        self.timer = self.create_timer(1.0, self.beep)
        self.get_logger().info('Beep Node Started!')

    def beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300000000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300000000)),  # 뽀
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300000000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300000000)),  # 뽀
        ]
        self.publisher.publish(msg)
        self.get_logger().info('삐뽀삐뽀!')

def main(args=None):
    rclpy.init(args=args)
    node = BeepNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()