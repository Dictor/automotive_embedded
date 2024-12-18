import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, ColorRGBA

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Pose

marker_x = [1.5]
marker_y = [0.0]

red = ColorRGBA()
red.r = 1.0
red.a = 1.0

green = ColorRGBA()
green.g = 1.0
green.a = 1.0

blue = ColorRGBA()
blue.b = 1.0
blue.a = 1.0

scale = Vector3()
scale.x = 0.15
scale.y = 0.15
scale.z = 0.15

class nVisual(Node):
    def __init__(self):
        super().__init__('visual')
        self.position_subscription_ = self.create_subscription(Float32MultiArray, 'position', self.position_callback, 10)
        self.publisher_ = self.create_publisher(MarkerArray, 'marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.position = [0.0, 0.0]

        self.rviz_marker = make_static_marker('marker', 1, marker_x[0], marker_y[0], 0, 'marker', red)
        self.rviz_reflu = make_static_marker('reflu', 2, 0, 0, 0, 'Left-Up ref', blue)
        self.rviz_refld = make_static_marker('refld', 3, 0, 1.5, 0, 'Left-Down ref', blue)
        self.rviz_refru = make_static_marker('refru', 4, 3, 0, 0, 'Right-Up ref', blue)
        self.rviz_refrd = make_static_marker('refrd', 5, 3, 1.5, 0, 'Right-Down ref', blue)

        self.rviz_jetson = Marker()
        self.rviz_jetson.header.frame_id = 'map'
        self.rviz_jetson.ns = 'jetson'
        self.rviz_jetson.id = 2
        self.rviz_jetson.type = Marker.CUBE
        self.rviz_jetson.action = Marker.MODIFY
        self.rviz_jetson.color = green
        self.rviz_jetson.scale = scale

    def timer_callback(self):
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        self.rviz_jetson.pose = pose
        self.rviz_jetson.text = "(%3f, %3f)" % (self.position[0], self.position[1])

        marker_array = MarkerArray()
        marker_array.markers.append(self.rviz_jetson)
        marker_array.markers.append(self.rviz_marker)
        marker_array.markers.append(self.rviz_reflu)
        marker_array.markers.append(self.rviz_refld)
        marker_array.markers.append(self.rviz_refru)
        marker_array.markers.append(self.rviz_refrd)
        self.publisher_.publish(marker_array)
        self.get_logger().info('Markers: "%s"' % marker_array)


    def position_callback(self, msg):
        self.position = msg.data

def make_static_marker(ns, id, x, y, z, text, color):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.ns = ns
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.MODIFY
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    marker.pose = pose
    marker.color = color
    marker.scale = scale
    marker.text = str(text)
    return marker
    

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