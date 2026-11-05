#!/usr/bin/env python3
"""Intake-boom controller (Issue #6) - the SG90 servo lowers / raises the intake hose and pump.

Speaks the same text protocol as robotic_arm.ino / the Node-RED arm flow:
    EXTEND        -> 90 deg (intake lowered into the river)
    RETRACT       -> 0 deg  (intake raised / stowed for driving)
    ANGLE:<0-90>  -> any angle in between

and moves with the same smooth interpolation as the firmware (1 deg every 20 ms).

  sub /waterdroid/arm/cmd      std_msgs/String
  sub /waterdroid/estop        std_msgs/Bool   (freezes the arm where it is)
  pub /arm_servo/cmd_pos       std_msgs/Float64 rad  -> Gazebo JointPositionController
  pub /waterdroid/arm/state    std_msgs/String JSON {"angle":..,"target":..,"moving":..}
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

STOW_DEG = 0
DEPLOY_DEG = 90


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.step_deg = self.declare_parameter('step_deg', 1.0).value
        self.step_period = self.declare_parameter('step_period_s', 0.02).value

        self.angle = float(STOW_DEG)
        self.target = float(STOW_DEG)
        self.estop = False

        self.cmd_pub = self.create_publisher(Float64, '/arm_servo/cmd_pos', 10)
        self.state_pub = self.create_publisher(String, '/waterdroid/arm/state', 10)
        self.create_subscription(String, '/waterdroid/arm/cmd', self._on_cmd, 10)
        self.create_subscription(Bool, '/waterdroid/estop', self._on_estop, 10)
        self.create_timer(self.step_period, self._step)
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info('Arm controller ready (EXTEND | RETRACT | ANGLE:0-90)')

    def _on_estop(self, msg):
        self.estop = msg.data
        if self.estop:
            self.target = self.angle

    def _on_cmd(self, msg):
        cmd = msg.data.strip().upper()
        if self.estop:
            self.get_logger().warn(f'Ignoring {cmd}: e-stop active')
            return
        if cmd == 'EXTEND':
            self.target = DEPLOY_DEG
        elif cmd == 'RETRACT':
            self.target = STOW_DEG
        elif cmd.startswith('ANGLE:'):
            value = cmd[6:]
            if not value.isdigit():
                self.get_logger().error('invalid angle command')
                return
            angle = int(value)
            if not STOW_DEG <= angle <= DEPLOY_DEG:
                self.get_logger().error('angle must be between 0 and 90')
                return
            self.target = float(angle)
        else:
            self.get_logger().error(f'unknown command {cmd}')
            return
        self.get_logger().info(f'arm target {self.target:.0f} deg')

    def _step(self):
        if abs(self.target - self.angle) >= self.step_deg:
            self.angle += self.step_deg if self.target > self.angle else -self.step_deg
        else:
            self.angle = self.target
        self.cmd_pub.publish(Float64(data=math.radians(self.angle)))

    def _publish_state(self):
        self.state_pub.publish(String(data=json.dumps({
            'angle': round(self.angle, 1),
            'target': round(self.target, 1),
            'moving': self.angle != self.target,
            'deployed': self.angle >= DEPLOY_DEG - 1,
            'stowed': self.angle <= STOW_DEG + 1,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
