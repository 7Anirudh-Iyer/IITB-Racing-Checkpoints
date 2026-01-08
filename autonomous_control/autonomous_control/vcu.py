import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VCUNode(Node):

    def __init__(self):
        super().__init__('vcu_node')

        self.subscription = self.create_subscription(
            String,
            'VCU2AI',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(String, 'AI2VCU', 10)

        self.timer = self.create_timer(1.0, self.check_ai_status)

        self.last_ai_time = None

    def listener_callback(self, msg):
        self.last_ai_time = self.get_clock().now()
        self.get_logger().info('I heard: "%s" → System is working' % msg.data)

    def check_ai_status(self):
        msg = String()

        if self.last_ai_time is None:
            msg.data = 'Not working'
            self.get_logger().warn(msg.data)
            self.publisher_.publish(msg)
            return

        time_diff = (self.get_clock().now() - self.last_ai_time).nanoseconds / 1e9

        if time_diff > 1.0:
            msg.data = 'Not working'
            self.get_logger().warn(msg.data)
        else:
            msg.data = 'Working'

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VCUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
