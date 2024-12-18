import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, ColorRGBA, Vector3

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from coord import marker_x, marker_y

class nVisual(Node):
    def __init__(self):
        super().__init__('visual')
        self.position_subscription_ = self.create_subscription(Float32MultiArray, 'position', self.position_callback, 10)
        self.publisher_ = self.create_publisher(Marker, 'marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.position = [0.0, 0.0]
        self.rviz_marker = Marker()
        self.rviz_marker.header.frame_id = 'map'
        self.rviz_marker.ns = 'marker'
        self.rviz_marker.id = 1
        self.rviz_marker.type = Marker.CUBE
        self.rviz_marker.action = Marker.MODIFY
        self.rviz_marker.points[0] = Point(marker_x, marker_y, 0)
        self.rviz_marker.color = ColorRGBA(1, 1, 0, 1)
        self.rviz_marker.scale = Vector3(0.2, 0.2, 0)

        self.rviz_jetson = Marker()
        self.rviz_jetson.header.frame_id = 'map'
        self.rviz_jetson.ns = 'jetson'
        self.rviz_jetson.id = 2
        self.rviz_jetson.type = Marker.CUBE
        self.rviz_jetson.action = Marker.MODIFY
        self.rviz_jetson.color = ColorRGBA(1, 0, 0, 1)
        self.rviz_jetson.scale = Vector3(0.2, 0.2, 0)

    def timer_callback(self):
        self.rviz_jetson.points[0] = Point(self.position[0], self.position[1], 0)
        self.get_logger().info('Position: "%s"' % self.position)
        self.publisher_.publish(self.rviz_jetson)
        self.publisher_.publish(self.rviz_marker)


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