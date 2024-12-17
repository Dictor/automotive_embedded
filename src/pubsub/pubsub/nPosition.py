import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32


class nPosition(Node):
    def __init__(self):
        super().__init__('position')
        self.dist_subscription_ = self.create_subscription(Float32, 'dist', self.dist_callback, 10)
        self.vector_subscription_ = self.create_subscription(Float32MultiArray, 'angle', self.vector_callback, 10)
        self.id_subscription_ = self.create_subscription(Int32, 'id', self.id_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.distance = -1.0
        self.id = -1
        self.vector = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0]

    def timer_callback(self):
        self.position = calc_position(self.vector, self.position, self.id)
        msg = Float32MultiArray()
        msg.data = self.position
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def vector_callback(self, msg):
        self.vector = msg.data
        return
    
    def dist_callback(self, msg):
        self.distance = msg.data
        return

    def id_callback(self, msg):
        self.id = msg.data
        return


def main(args=None):
    rclpy.init(args=args)
    node = nPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def calc_position(vector, position, id):
    # Fill here
    return [0.0, 0.0]


if __name__ == '__main__':
    main()