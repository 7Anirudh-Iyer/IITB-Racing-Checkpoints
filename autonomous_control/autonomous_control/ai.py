import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AINode(Node):

    def __init__(self):
        super().__init__('ai_node')
        self.publisher_ = self.create_publisher(String, 'VCU2AI', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'AI Functioning'
        self.publisher_.publish(msg)
        self.get_logger().info('Data sent')


def main(args=None):
    rclpy.init(args=args)
    node = AINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
