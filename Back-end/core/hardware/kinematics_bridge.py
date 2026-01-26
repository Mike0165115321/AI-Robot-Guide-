#!/usr/bin/env python3
"""
Kinematics Bridge Node
=======================
Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes to /wheel*_command topics
for compatibility with the legacy base_control.ino firmware.

Run this alongside ros2_bridge.py for full control chain.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class KinematicsBridge(Node):
    def __init__(self):
        super().__init__('kinematics_bridge')

        # Robot Parameters
        self.max_rpm_limit = 100.0  # Absolute Max hardware capability
        self.current_speed_scale = 0.5 # Default 50% speed

        # Ramping State (Subtle smoothness)
        self.current_rpm = [0.0, 0.0, 0.0, 0.0]
        self.target_rpm = [0.0, 0.0, 0.0, 0.0]
        # Ramp step: 2.0 per 50ms = 40 RPM/s (Ultra Smooth)
        self.ramp_step = 2.0 

        # Publishers
        self.pub_w1 = self.create_publisher(Float32, 'wheel1_command', 10)
        self.pub_w2 = self.create_publisher(Float32, 'wheel2_command', 10)
        self.pub_w3 = self.create_publisher(Float32, 'wheel3_command', 10)
        self.pub_w4 = self.create_publisher(Float32, 'wheel4_command', 10)

        # Subscriber for CMD_VEL
        self.sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            
        # Timer for Ramping
        self.timer = self.create_timer(0.05, self.update_and_publish)

        self.last_cmd_time = 0.0 # Time of last received command
        self.cmd_timeout = 1   # Seconds before auto-stop

        self.get_logger().info('🚗 Kinematics Bridge (Smooth-Step=5.0, Timeout=0.5s) started.')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # Debug: Check what value we are actingally receiving
        if abs(vx) > 0.01:
            self.get_logger().info(f"Rx cmd_vel: vx={vx:.2f} (Speed Scale Check)")

        # Calculate Scale based on Speed Slider (Received from hardware_service usually, but for now let's set a reliable base)
        # Wait, hardware_service sends RAW 0-1.
        # We need to scaling factor.
        
        # Let's use a fixed High Base, and rely on hardware_service to send pre-scaled values?
        # No, user said speed slider didn't work. hardware_service was delivering scaled values 0-0.5, but we removed that multiplier.
        
        # We should put the multiplier BACK in hardware_service but make it HIGHER.
        # But to be safe, let's handle smooth ramping here.
        
        # Base RPM for 100% input
        BASE_RPM = 120.0 

        rpm_fl = (vx - vy - wz) * BASE_RPM
        rpm_fr = (vx + vy + wz) * BASE_RPM
        rpm_rl = (vx + vy - wz) * BASE_RPM
        rpm_rr = (vx - vy + wz) * BASE_RPM
        
        self.target_rpm = [rpm_fl, rpm_fr, rpm_rl, rpm_rr]
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def update_and_publish(self):
        # 0. Safety Timeout Check
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_cmd_time) > self.cmd_timeout:
            # No command received for > 0.5s. Force stop target.
            # This handles the case where user releases keys and ros2_bridge stops publishing.
            self.target_rpm = [0.0, 0.0, 0.0, 0.0]

        # Soft Start & Smooth Stop Logic
        
        MIN_MOVING_RPM = 15.0 
        
        # Ramp Parameters
        ACCEL_STEP = 2.0  # Smooth start
        DECEL_FACTOR = 0.95 # Exponential Decay (Multiply by 0.95 each cycle)

        for i in range(4):
            target = self.target_rpm[i]
            current = self.current_rpm[i]
            
            # 1. Deadzone Jump (Only when starting from stop AND target is significant)
            if abs(target) > 0.1 and abs(current) < MIN_MOVING_RPM:
                sign = 1.0 if target > 0 else -1.0
                self.current_rpm[i] = MIN_MOVING_RPM * sign
                continue

            # 2. Ramping Logic
            err = target - current
            
            # Check if we are Decelerating (Moving towards 0 or moving opposite to velocity)
            is_decel = (abs(target) < abs(current)) or (target * current < 0)
            
            if is_decel:
                 # Exponential Decay: Multiply current speed by factor
                 # This creates a very natural "Coasting" feel
                 # If target is 0 (Stopping), just multiply
                 # If changing speed but still decel, linear ramp might be better to reach target?
                 # Let's use Decay for stopping (Target=0), Linear for changing speed
                 
                 if abs(target) < 0.1:
                      # Coasting to stop
                      self.current_rpm[i] *= DECEL_FACTOR
                      
                      # Snap to 0 if very slow
                      if abs(self.current_rpm[i]) < 0.5:
                          self.current_rpm[i] = 0.0
                 else:
                      # Decelerating to a non-zero speed (e.g. slowing down)
                      # Use Linear Ramp (Step 2.0)
                      step = 2.0
                      if abs(err) > step:
                          self.current_rpm[i] += step if err > 0 else -step
                      else:
                          self.current_rpm[i] = target
            else:
                # Accelerating (Linear Ramp)
                step = ACCEL_STEP
                if abs(err) > step:
                    self.current_rpm[i] += step if err > 0 else -step
                else:
                    self.current_rpm[i] = target

        # Publish
        self.pub_w1.publish(Float32(data=self.current_rpm[0]))
        self.pub_w2.publish(Float32(data=self.current_rpm[1]))
        self.pub_w3.publish(Float32(data=self.current_rpm[2]))
        self.pub_w4.publish(Float32(data=self.current_rpm[3]))


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
