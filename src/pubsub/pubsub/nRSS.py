# for ros
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

# for ble
import asyncio
from bleak import BleakScanner
import math


class nRSS(Node):
    def __init__(self):
        super().__init__('rss')
        self.publisher_ = self.create_publisher(Float32, 'dist', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.distance = -1.0

    def timer_callback(self):
        self.distance = asyncio.run(self.read_distance())
        if self.distance < 0:
            self.get_logger().error('failed to read distance')
            return

        msg = Float32()
        msg.data = self.distance
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f"' % msg.data)

    async def read_distance(self):
        rssi = await get_rssi()
        if rssi is not None:
            distance = rssi_to_distance(rssi)
            return distance
        else:
            return -1

def main(args=None):
    rclpy.init(args=args)
    node = nRSS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

async def get_rssi(target_name="HMSoft"):
    devices = await BleakScanner.discover(timeout=5.0)
    for device in devices:
        if target_name in (device.name or ""):
            return device.rssi
    return None

def rssi_to_distance(rssi, tx_power=-59, n=3):
    return 10 ** ((tx_power - rssi) / (10 * n)) / 5 


if __name__ == '__main__':
    main()