import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32
import math

class nPosition(Node):
    def __init__(self):
        super().__init__('position')
        self.dist_subscription_ = self.create_subscription(Float32, 'dist', self.dist_callback, 10)
        self.vector_subscription_ = self.create_subscription(Float32MultiArray, 'angle', self.vector_callback, 10)
        self.id_subscription_ = self.create_subscription(Int32, 'id', self.id_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.marker_x = [1.5]
        self.marker_y = [0]

        self.distance = -1.0
        self.id = -1
        self.angle = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0]

    def timer_callback(self):
        self.get_logger().info('all subs: dist "%s" id "%d" vector "%s"' % (self.distance, self.id, self.vector))
        self.position = self.calc_position(self.angle, self.distance, self.id)
        msg = Float32MultiArray()
        msg.data = self.position
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def vector_callback(self, msg):
        self.angle = msg.data
        return
    
    def dist_callback(self, msg):
        self.distance = msg.data
        return

    def id_callback(self, msg):
        self.id = msg.data
        return

    def calc_position(self, angle, distance, id):
        rad = angle[1] / 180
        x = self.marker_x[id] + distance * math.sin(rad)
        y = self.marker_y[id] + distance * math.cos(rad)
        return (float(x), float(y))
    
def main(args=None):
    rclpy.init(args=args)
    node = nPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


    return [0.0, 0.0]


if __name__ == '__main__':
    main()