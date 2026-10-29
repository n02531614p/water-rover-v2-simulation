import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class MissionNavigator(Node):
    def __init__(self):
        super().__init__("mission_navigator")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        
        # Exact world locations: River at 9.5m
        # OUTSIDE_SWING (7.2, -1.0) routes the rover cleanly around Home 3 without clipping its radius
        self.waypoints = [
            {"name": "RIVER_INTAKE", "x": 9.5, "y": 0.0, "wait": 3.0, "thresh": 0.25, "action": "Collecting Raw Water"},
            {"name": "BAY_1_HOME1", "x": 6.0, "y": 2.0, "wait": 3.0, "thresh": 0.18, "action": "Offloading to Home 1"},
            {"name": "BAY_2_HOME2", "x": 6.0, "y": -2.0, "wait": 3.0, "thresh": 0.18, "action": "Offloading to Home 2"},
            {"name": "OUTSIDE_SWING", "x": 7.2, "y": -1.0, "wait": 0.0, "thresh": 0.20, "action": "Swinging outside Home 3"},
            {"name": "BAY_3_HOME3", "x": 6.0, "y": 0.0, "wait": 3.0, "thresh": 0.15, "action": "Offloading to Home 3"},
            {"name": "RETURN_PLANT", "x": 0.0, "y": 0.0, "wait": 2.0, "thresh": 0.25, "action": "Docked at Plant Base"}
        ]
        self.current_wp_idx = 0
        self.start_wait_time = None
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            return
            
        cmd = Twist()
        
        if self.current_wp_idx >= len(self.waypoints):
            self.cmd_pub.publish(cmd)
            self.get_logger().info("FULL MISSION COMPLETE!")
            return

        target = self.waypoints[self.current_wp_idx]
        dx = target["x"] - self.x
        dy = target["y"] - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        arrival_thresh = target.get("thresh", 0.18)

        if dist > arrival_thresh:
            self.start_wait_time = None
            target_yaw = math.atan2(dy, dx)
            yaw_err = target_yaw - self.yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            if abs(yaw_err) > 0.35:
                cmd.angular.z = 0.5 if yaw_err > 0 else -0.5
            else:
                cmd.linear.x = max(0.18, min(0.35, 0.5 * dist))
                cmd.angular.z = 0.5 * yaw_err
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            if target["wait"] == 0.0:
                self.current_wp_idx += 1
                return

            if self.start_wait_time is None:
                self.start_wait_time = time.time()
                self.get_logger().info(f"STATUS: Arrived at {target["name"]} - {target["action"]}")
            
            if time.time() - self.start_wait_time >= target["wait"]:
                self.get_logger().info(f"STATUS: Completed {target["name"]}. Moving to next destination...")
                self.current_wp_idx += 1
                self.start_wait_time = None

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
