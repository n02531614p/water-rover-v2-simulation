#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class ArmDeliveryFSM(Node):
    def __init__(self):
        super().__init__('arm_delivery_fsm')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.state = 'NAV_TO_WELL'
        
        self.homes = [
            {"name": "Home Stand 1", "x": 3.0, "y": 3.0},
            {"name": "Home Stand 2", "x": 5.0, "y": 2.5},
            {"name": "Home Stand 3", "x": 4.0, "y": -3.0},
            {"name": "Home Stand 4", "x": -2.0, "y": -2.5},
            {"name": "Home Stand 5", "x": -3.0, "y": -2.5}
        ]
        
        self.current_home_idx = 0
        self.homes_served_this_batch = 0
        self.fill_time = 0
        
        self.get_logger().info("=== Arm-Articulated Delivery Controller Active ===")
        self.timer = self.create_timer(0.1, self.run_fsm)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def navigate_to(self, target_x, target_y):
        msg = Twist()
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))

        if abs(yaw_err) > 0.2:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 if yaw_err > 0 else -0.5
        else:
            msg.linear.x = min(0.4, 0.6 * dist)
            msg.angular.z = 0.5 * yaw_err

        self.cmd_pub.publish(msg)
        return dist

    def run_fsm(self):
        if self.state == 'NAV_TO_WELL':
            dist = self.navigate_to(7.0, 0.0)
            if dist < 0.6:
                self.cmd_pub.publish(Twist())
                self.get_logger().info("[AT WELL] Pumping 1000 mL... Filtering & Boiling Onboard...")
                self.state = 'PROCESS_ONBOARD'
                self.fill_time = 0
                self.homes_served_this_batch = 0

        elif self.state == 'PROCESS_ONBOARD':
            if self.fill_time < 25:
                self.fill_time += 1
            else:
                current_home = self.homes[self.current_home_idx]
                self.get_logger().info(f"[TREATED] Tank full! Navigating to {current_home['name']}...")
                self.state = 'NAV_TO_HOME'

        elif self.state == 'NAV_TO_HOME':
            target_home = self.homes[self.current_home_idx]
            dist = self.navigate_to(target_home["x"], target_home["y"])
            
            if dist < 0.6:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"[DELIVERY] Arrived at {target_home['name']}!")
                self.get_logger().info("[ARM] Extending telescopic nozzle to household stand...")
                time.sleep(1.5)
                self.get_logger().info("[DISPENSING] Dispensing 300 mL treated water...")
                time.sleep(2.0)
                self.get_logger().info("[ARM] Retracting telescopic nozzle into chassis...")
                time.sleep(1.0)
                
                self.homes_served_this_batch += 1
                self.current_home_idx = (self.current_home_idx + 1) % len(self.homes)
                
                if self.homes_served_this_batch >= 3:
                    self.get_logger().info("=== Tank Low (3 Homes Served). Returning to Well ===")
                    self.state = 'NAV_TO_WELL'
                else:
                    next_home = self.homes[self.current_home_idx]
                    self.get_logger().info(f"=== Water remaining. Proceeding to {next_home['name']} ===")

def main():
    rclpy.init()
    node = ArmDeliveryFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
