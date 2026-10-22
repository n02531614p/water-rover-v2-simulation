#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class WaterProcessFSM(Node):
    def __init__(self):
        super().__init__('water_process_fsm')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.state = 'IDLE'
        self.batch_count = 1
        self.fill_time = 0
        
        self.get_logger().info("=== Dynamic Target Vector Navigation FSM Initialized ===")
        self.timer = self.create_timer(0.1, self.run_fsm)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Quaternion to Yaw conversion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def navigate_to_target(self, target_x, target_y):
        msg = Twist()
        
        # Calculate distance to goal
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        
        # Calculate desired heading directly toward target
        desired_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))

        # First align orientation if error > 0.15 rad (~8 deg)
        if abs(yaw_error) > 0.15:
            msg.linear.x = 0.0
            msg.angular.z = 0.6 if yaw_error > 0 else -0.6
        else:
            # Drive forward toward target with proportional steering correction
            msg.linear.x = min(0.4, 0.5 * distance)
            msg.angular.z = 1.0 * yaw_error

        self.cmd_pub.publish(msg)
        return distance

    def run_fsm(self):
        if self.state == 'IDLE':
            self.get_logger().info(f"[BATCH #{self.batch_count}] Target locked: Water Source (6.4, 0.0)")
            self.state = 'NAV_TO_SOURCE'

        elif self.state == 'NAV_TO_SOURCE':
            # Drive to Water Source at x = 6.4, y = 0.0
            dist = self.navigate_to_target(6.4, 0.0)
            if dist < 0.2:
                self.stop()
                self.get_logger().info("[STATE: AT_SOURCE] Vehicle aligned at water well. Starting pump...")
                self.state = 'COLLECTING'
                self.fill_time = 0

        elif self.state == 'COLLECTING':
            if self.fill_time < 35:  # 3.5 seconds fill time to reach 350 mL
                self.fill_time += 1
                self.get_logger().info(f"[PUMPING] Tank filling... ({self.fill_time * 10} / 350 mL)")
            else:
                self.get_logger().info("[PUMPING COMPLETE] 350 mL full capacity reached. Reorienting to Treatment Dock...")
                self.state = 'NAV_TO_DOCK'

        elif self.state == 'NAV_TO_DOCK':
            # Drive directly back to Dock at origin x = 0.0, y = 0.0
            dist = self.navigate_to_target(0.0, 0.0)
            if dist < 0.25:
                self.stop()
                self.get_logger().info("[DOCKED] Vehicle successfully docked at Treatment Station.")
                self.state = 'FILTER_AND_BOIL'

        elif self.state == 'FILTER_AND_BOIL':
            self.get_logger().info("[TREATMENT] Filtering sediment (>50% turbidity reduction)...")
            time.sleep(2)
            self.get_logger().info("[TREATMENT] Heating to rolling boil (>100 C)...")
            time.sleep(3)
            self.get_logger().info(f"=== Batch #{self.batch_count} Complete! Telemetry Logged. ===")
            
            # Reset for next batch automatically
            self.batch_count += 1
            self.get_logger().info(f"=== Initializing Batch #{self.batch_count} ===")
            self.state = 'IDLE'

def main(args=None):
    rclpy.init(args=args)
    node = WaterProcessFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
