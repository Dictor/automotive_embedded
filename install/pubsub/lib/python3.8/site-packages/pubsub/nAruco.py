import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray  


class nAruco(Node):
    def __init__(self):
        super().__init__('aruco')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'angle', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.vector = [0.0, 0.0, 0.0]

    def timer_callback(self):
        self.vector = read_vector()
        msg = Float32MultiArray()
        msg.data = self.vector
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = nAruco()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def read_vector():
    # Fill here
    return [0.0, 0.0, 0.0]


if __name__ == '__main__':
    main()