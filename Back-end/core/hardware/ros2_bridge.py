#!/usr/bin/python3
import socket
import json
import signal
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
            
            self.get_logger().info(f"Received UDP: vx={vx}, wz={wz}")
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
    
    def cleanup(self):
        """Clean up resources before shutdown."""
        try:
            self.sock.close()
        except:
            pass

# Global node reference for signal handler
_node = None

def signal_handler(signum, frame):
    """Handle SIGTERM/SIGINT gracefully."""
    global _node
    if _node:
        _node.get_logger().info("🛑 Received shutdown signal, cleaning up...")
        _node.cleanup()
    sys.exit(0)

def main(args=None):
    global _node
    
    # Register signal handlers
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.init(args=args)
    _node = TeleopBridge()
    
    try:
        rclpy.spin(_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        _node.cleanup()
        _node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Ignore shutdown errors (already shutdown, etc.)
            pass

if __name__ == '__main__':
    main()

