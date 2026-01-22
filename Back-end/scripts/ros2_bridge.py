#!/usr/bin/python3
import socket
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# UDP Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 9999

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('web_teleop_bridge')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info(f"🌉 ROS2 Teleop Bridge Started. Listening on UDP {UDP_PORT}")
        
        # Setup UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)  # Non-blocking mode

        # Timer to check UDP
        self.create_timer(0.01, self.check_udp) # 100Hz check

    def check_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            cmd = json.loads(data.decode('utf-8'))
            
            vx = float(cmd.get("vx", 0.0))
            wz = float(cmd.get("wz", 0.0))
            
            self.publish_velocity(vx, wz)
            
        except BlockingIOError:
            pass # No data
        except Exception as e:
            self.get_logger().error(f"Error parsing UDP: {e}")

    def publish_velocity(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
