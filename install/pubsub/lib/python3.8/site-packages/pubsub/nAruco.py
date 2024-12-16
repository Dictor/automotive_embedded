import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int32  


class nAruco(Node):
    def __init__(self):
        super().__init__('aruco')
        self.angle_publisher_ = self.create_publisher(Float32MultiArray, 'angle', 10)
        self.id_publisher_ = self.create_publisher(Int32, 'id', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.vector = [0.0, 0.0, 0.0]
        self.id = -1

    def timer_callback(self):
        marker = read_marker()
        self.vector = marker[0]
        self.id = marker[1]

        msg = Float32MultiArray()
        msg.data = self.vector
        self.angle_publisher_.publish(msg)
        self.get_logger().info('Publishing angle: "%s"' % msg.data)

        msg = Int32()
        msg.data = self.id
        self.id_publisher_.publish(msg)
        self.get_logger().info('Publishing id: %d' % msg.data)



def main(args=None):
    rclpy.init(args=args)
    node = nAruco()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def read_marker():
    # Fill here
    return ([0.0, 0.0, 0.0], 0)


if __name__ == '__main__':
    main()