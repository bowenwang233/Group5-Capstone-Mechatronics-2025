# PhysicalTwin/LidarPublisher.py
#!/usr/bin/env python3
import json, zmq, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher_zmq')
        self.sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        ctx = zmq.Context.instance()
        self.sock = ctx.socket(zmq.PUB)
        self.sock.bind('tcp://*:5560')  # Pi publishes here
        self.get_logger().info('Publishing /scan to tcp://*:5560')

    def _on_scan(self, msg: LaserScan):
        payload = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities) if msg.intensities else []
        }
        self.sock.send_json(payload)

def main():
    rclpy.init()
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()