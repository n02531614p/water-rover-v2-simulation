#!/usr/bin/env python3
# Ground-truth odometry version of the Waterdroid delivery mission.
#
# Why this file exists:
#   The Gazebo DiffDrive wheel odometry can report motion that the body did not
#   actually make when the wheels slip. This mission uses the ground-truth pose
#   published by the Gazebo OdometryPublisher instead. The first pose is treated
#   as the origin, so the route coordinates are relative to the spawn pose.
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MissionNavigator(Node):
    def __init__(self):
        super().__init__('mission_navigator')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10
        )
        self.timer = self.create_timer(0.02, self.control_loop)

        self.state = 0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0

        # Ground-truth odometry starts at the spawn pose. Subtract the first
        # pose so the mission always starts at (0, 0, 0), like the old odom.
        self.origin_x = None
        self.origin_y = None
        self.origin_yaw = None

        self.action_start_sim_time = None
        self.PAUSE_DURATION_SEC = 5.0

        self.get_logger().info(
            '=== Waterdroid 4WD Navigator (ground-truth odom) Active ==='
        )

    def get_sim_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.origin_x is None:
            self.origin_x = x
            self.origin_y = y
            self.origin_yaw = yaw
            self.get_logger().info(
                f'Ground-truth origin set: x={x:.2f} y={y:.2f} yaw={yaw:.2f}'
            )

        self.pose_x = x - self.origin_x
        self.pose_y = y - self.origin_y
        self.yaw = math.atan2(
            math.sin(yaw - self.origin_yaw),
            math.cos(yaw - self.origin_yaw)
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def drive_towards(self, target_x, target_y, speed=0.35):
        dx = target_x - self.pose_x
        dy = target_y - self.pose_y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(
            math.sin(target_yaw - self.yaw),
            math.cos(target_yaw - self.yaw)
        )

        cmd = Twist()
        if abs(yaw_error) > 0.35:
            cmd.angular.z = 1.0 if yaw_error > 0 else -1.0
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(speed, max(0.15, 0.5 * dist))
            cmd.angular.z = 1.0 * yaw_error

        self.cmd_pub.publish(cmd)
        return dist

    def control_loop(self):
        cmd = Twist()
        now_sec = self.get_sim_time_sec()

        # Wait for the first ground-truth pose before commanding motion.
        if self.origin_x is None:
            self.stop()
            return

        # STATE 0: DRIVE TO RIVERBANK
        # Stop with the front bumper near the bank, not over it.
        if self.state == 0:
            if self.pose_x < 9.6:
                cmd.linear.x = 0.35
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
            else:
                self.stop()
                self.get_logger().info(
                    f'>>> ARRIVED AT RIVERBANK at x={self.pose_x:.2f}! '
                    'PAUSING 5s TO PUMP WATER... <<<'
                )
                self.action_start_sim_time = now_sec
                self.state = 1

        # STATE 1: PUMP WATER (5s pause)
        elif self.state == 1:
            if now_sec - self.action_start_sim_time < self.PAUSE_DURATION_SEC:
                self.stop()
            else:
                self.get_logger().info(
                    '>>> WATER TANK FULL! Reversing off bank... <<<'
                )
                self.action_start_sim_time = now_sec
                self.state = 2

        # STATE 2: REVERSE DISENGAGE
        # Back up farther so all four wheels are on flat ground for the pivot.
        elif self.state == 2:
            if now_sec - self.action_start_sim_time < 2.5:
                cmd.linear.x = -0.25
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
            else:
                self.stop()
                self.action_start_sim_time = now_sec
                self.get_logger().info(
                    f'>>> EXECUTING 180 DEGREE PIVOT at x={self.pose_x:.2f}, '
                    f'yaw={self.yaw:.2f} <<<'
                )
                self.state = 3

        # STATE 3: CLOSED-LOOP 180 PIVOT (facing west: yaw = pi)
        elif self.state == 3:
            # Let the reverse motion settle before turning.
            if now_sec - self.action_start_sim_time < 0.5:
                self.stop()
                return

            target_yaw = math.pi
            yaw_error = math.atan2(
                math.sin(target_yaw - self.yaw),
                math.cos(target_yaw - self.yaw)
            )

            if abs(yaw_error) > 0.15:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6 if yaw_error > 0 else -0.6
                self.cmd_pub.publish(cmd)
            else:
                self.stop()
                self.get_logger().info(
                    f'STATE 4: 180 Pivot Complete (yaw={self.yaw:.2f})! '
                    'Heading to Homestead 1...'
                )
                self.state = 4

        # STATE 4: DRIVE TO BAY 1 (6.0, 3.0)
        elif self.state == 4:
            if self.drive_towards(6.0, 3.0) < 0.45:
                self.stop()
                self.get_logger().info(
                    '>>> PAUSING 5s: OFFLOADING WATER AT HOMESTEAD 1... <<<'
                )
                self.action_start_sim_time = now_sec
                self.state = 5

        # STATE 5: OFFLOAD BAY 1
        elif self.state == 5:
            if now_sec - self.action_start_sim_time < self.PAUSE_DURATION_SEC:
                self.stop()
            else:
                self.get_logger().info('STATE 6: Driving to Homestead 2...')
                self.state = 6

        # STATE 6: DRIVE TO BAY 2 (6.0, -3.0)
        elif self.state == 6:
            if self.drive_towards(6.0, -3.0) < 0.45:
                self.stop()
                self.get_logger().info(
                    '>>> PAUSING 5s: OFFLOADING WATER AT HOMESTEAD 2... <<<'
                )
                self.action_start_sim_time = now_sec
                self.state = 7

        # STATE 7: OFFLOAD BAY 2
        elif self.state == 7:
            if now_sec - self.action_start_sim_time < self.PAUSE_DURATION_SEC:
                self.stop()
            else:
                self.get_logger().info(
                    'STATE 8: Driving onto Homestead 3 Offloading Pad...'
                )
                self.state = 8

        # STATE 8: DRIVE TO BAY 3 PAD (-3.0, -3.5)
        elif self.state == 8:
            if self.drive_towards(-3.0, -3.5) < 0.45:
                self.stop()
                self.get_logger().info(
                    '>>> PAUSING 5s: OFFLOADING WATER AT HOMESTEAD 3... <<<'
                )
                self.action_start_sim_time = now_sec
                self.state = 9

        # STATE 9: OFFLOAD BAY 3
        elif self.state == 9:
            if now_sec - self.action_start_sim_time < self.PAUSE_DURATION_SEC:
                self.stop()
            else:
                self.get_logger().info(
                    'STATE 10: Returning to Depot Base...'
                )
                self.state = 10

        # STATE 10: RETURN DEPOT (0.0, 0.0)
        elif self.state == 10:
            if self.drive_towards(0.0, 0.0) < 0.4:
                self.stop()
                self.get_logger().info(
                    '=== MISSION COMPLETE: Waterdroid parked at Depot Base ==='
                )
                self.state = 11

        # STATE 11: COMPLETE
        elif self.state == 11:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
