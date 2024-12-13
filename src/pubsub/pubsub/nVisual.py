import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


class nVisual(Node):
    def __init__(self):
        super().__init__('visual')
        self.position_subscription_ = self.create_subscription(Float32MultiArray, 'position', self.position_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.position = [0.0, 0.0]

    def timer_callback(self):
        self.get_logger().info('Position: "%s"' % self.position)

    def position_callback(self, msg):
        self.position = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = nVisual()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def read_distance():
    # Fill here
    return 0.0


if __name__ == '__main__':
    main()