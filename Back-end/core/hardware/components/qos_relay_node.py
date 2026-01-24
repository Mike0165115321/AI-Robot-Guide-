#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        
        # QoS for Subscriber (Match LIDAR: Best Effort)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for Publisher (Match Nav2: Reliable)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.get_logger().info("🚀 Scan Relay Started: [/scan (Best Effort)] -> [/scan_reliable (Reliable)]")
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos_sub)
        self.pub = self.create_publisher(LaserScan, '/scan_reliable', qos_pub)

    def cb(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
