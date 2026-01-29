#!/usr/bin/env python3
"""
Mecanum Odometry Node
=====================
Custom odometry calculation for robot with specific kinematics parameters:
- Track Width: 0.33m (Lx = 0.165)
- Wheel Base: 0.24m (Ly = 0.12)
- Encoder: 890 ticks/rev
- Wheel Radius: 0.05m (Assumed 10cm Diameter)

Subscribes to:
- /m1_tick, /m2_tick, /m3_tick, /m4_tick (std_msgs/Int32)

Publishes:
- /odom (nav_msgs/Odometry)
- /tf (odom -> base_link)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_node')

        # --- Robot Parameters ---
        self.wheel_radius = 0.05      # meters (Assumed 10cm diameter)
        self.lx = 0.165               # meters (Half Track Width: 33cm / 2)
        self.ly = 0.12                # meters (Half Wheel Base: 24cm / 2)
        self.ticks_per_rev = 890.0    # ticks per revolution

        # Calculated Constants
        self.rad_per_tick = (2 * math.pi) / self.ticks_per_rev
        self.geom_factor = self.lx + self.ly

        # State Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.prev_ticks = [None, None, None, None] # FL, FR, RL, RR
        self.current_ticks = [0, 0, 0, 0]
        
        self.last_time = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Int32, 'm1_tick', lambda msg: self.update_tick(0, msg), 10) # FL
        self.create_subscription(Int32, 'm2_tick', lambda msg: self.update_tick(1, msg), 10) # FR
        self.create_subscription(Int32, 'm3_tick', lambda msg: self.update_tick(2, msg), 10) # RL
        self.create_subscription(Int32, 'm4_tick', lambda msg: self.update_tick(3, msg), 10) # RR

        # Timer for calculation loop (e.g., 20Hz)
        self.create_timer(0.05, self.update_odometry)

        self.get_logger().info('✅ Mecanum Odometry Node Started')
        self.get_logger().info(f'   - Params: Lx={self.lx}, Ly={self.ly}, R={self.wheel_radius}, PPR={self.ticks_per_rev}')

    def update_tick(self, index, msg):
        self.current_ticks[index] = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        
        # Check if we have initialized previous ticks
        if any(t is None for t in self.prev_ticks):
            self.prev_ticks = list(self.current_ticks)
            self.last_time = current_time
            return

        # Calculate Delta Ticks
        delta_ticks = [c - p for c, p in zip(self.current_ticks, self.prev_ticks)]
        self.prev_ticks = list(self.current_ticks)

        # delta_ticks mapping based on standard mecanum layout:
        # 0: FL (m1), 1: FR (m2), 2: RL (m3), 3: RR (m4)
        # Verify direction: Usually forward motion produces +ticks on all.
        
        # Calculate Wheel Rotations (Radians)
        w = [d * self.rad_per_tick for d in delta_ticks]
        
        # Mecanum Forward Kinematics (Displacement)
        # dx_robot = (w_fl + w_fr + w_rl + w_rr) * R / 4
        # dy_robot = (-w_fl + w_fr + w_rl - w_rr) * R / 4
        # dth_robot = (-w_fl + w_fr - w_rl + w_rr) * R / (4 * (Lx + Ly))

        dx_r = (w[0] + w[1] + w[2] + w[3]) * self.wheel_radius / 4.0
        dy_r = (-w[0] + w[1] + w[2] - w[3]) * self.wheel_radius / 4.0
        dth_r = (-w[0] + w[1] - w[2] + w[3]) * self.wheel_radius / (4.0 * self.geom_factor)

        # Update Sine/Cosine for integration
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return # Avoid division by zero on startup or clock jump

        delta_x = (dx_r * math.cos(self.th) - dy_r * math.sin(self.th))
        delta_y = (dx_r * math.sin(self.th) + dy_r * math.cos(self.th))
        
        self.x += delta_x
        self.y += delta_y
        self.th += dth_r

        # Publish TF
        q = self.euler_to_quaternion(0, 0, self.th)
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        # Velocity (Relative to Robot Frame)
        odom.twist.twist.linear.x = dx_r / dt
        odom.twist.twist.linear.y = dy_r / dt
        odom.twist.twist.angular.z = dth_r / dt

        self.odom_pub.publish(odom)
        
        self.last_time = current_time

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
