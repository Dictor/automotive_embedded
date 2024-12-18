import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3

marker_x = [1.5]
marker_y = [0.0]

red = ColorRGBA()
red.r = 1.0
red.a = 1.0

green = ColorRGBA()
green.r = 1.0
green.a = 1.0

scale = Vector3()
scale.x = 1.0
scale.y = 1.0
scale.z = 1.0

class nVisual(Node):
    def __init__(self):
        super().__init__('visual')
        self.position_subscription_ = self.create_subscription(Float32MultiArray, 'position', self.position_callback, 10)
        self.publisher_ = self.create_publisher(MarkerArray, 'marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.position = [0.0, 0.0]

        self.rviz_marker = Marker()
        self.rviz_marker.header.frame_id = 'map'
        self.rviz_marker.ns = 'marker'
        self.rviz_marker.id = 1
        self.rviz_marker.type = Marker.CUBE
        self.rviz_marker.action = Marker.MODIFY
        point = Point()
        point.x = marker_x[0]
        point.y = marker_y[0]
        point.z = 0.0
        self.rviz_marker.points.append(point)
        self.rviz_marker.color = red
        self.rviz_marker.scale = scale

        self.rviz_jetson = Marker()
        self.rviz_jetson.header.frame_id = 'map'
        self.rviz_jetson.ns = 'jetson'
        self.rviz_jetson.id = 2
        self.rviz_jetson.type = Marker.CUBE
        self.rviz_jetson.action = Marker.MODIFY
        self.rviz_jetson.points.append(point)
        self.rviz_jetson.color = green
        self.rviz_jetson.scale = scale

    def timer_callback(self):
        point = Point()
        point.x = self.position[0]
        point.y = self.position[1]
        point.z = 0.0
        self.rviz_jetson.points[0] = point
        self.get_logger().info('Position: "%s"' % self.position)

        marker_array = MarkerArray()
        marker_array.markers.append(self.rviz_jetson)
        marker_array.markers.append(self.rviz_marker)
        self.publisher_.publish(self.marker_array)


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