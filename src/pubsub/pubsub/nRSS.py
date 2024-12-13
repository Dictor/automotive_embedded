import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class nRSS(Node):
    def __init__(self):
        super().__init__('rss')
        self.publisher_ = self.create_publisher(Float32, 'dist', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.distance = -1.0

    def timer_callback(self):
        self.distance = read_distance()
        if self.distance < 0:
            return

        msg = Float32()
        msg.data = self.distance
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = nRSS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def read_distance():
    # Fill here
    return 0.0


if __name__ == '__main__':
    main()