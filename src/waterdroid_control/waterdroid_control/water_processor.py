#!/usr/bin/env python3
"""Simulated water-treatment train of the Waterdroid rover.

Gazebo cannot simulate the water itself, so this node models the four 5 V pumps,
the three stages and their level sensors with the same rules the ESP32 firmware
uses (firmware/waterdroid_esp32). It lets the mission, the dashboard and the
Node-RED flows be exercised end-to-end in simulation.

Hardware it stands in for (relay channel in brackets):
  Issue #1  5 V submersible intake pump        [CH1] -> Stage 1 raw tank (on the SG90 intake boom)
  Issue #5  5 V transfer pump                   [CH2] filtration column -> Stage 3 delivery tank
            5 V chlorine dosing pump            [CH3] -> Stage 3
  Issue #5  Stage 2 gravity filtration column   (no pump, gravity flow S1 -> S3)
  Issue #8  5 V delivery pump                   [CH4] Stage 3 -> homestead container

Interface
  sub  /waterdroid/cmd          std_msgs/String   commands, see _on_cmd()
  sub  /waterdroid/estop        std_msgs/Bool     True = all pumps off and latched
  sub  /ground_truth/odom       nav_msgs/Odometry used to know if the hose is in the river
  sub  /joint_states            sensor_msgs/JointState  intake-boom angle (water only with the boom lowered)
  pub  /waterdroid/process_state std_msgs/String  JSON, 5 Hz
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

RELAY_NAMES = ('intake_pump', 'transfer_pump', 'chlorine_pump', 'delivery_pump')


class WaterProcessor(Node):
    def __init__(self):
        super().__init__('water_processor')

        p = self.declare_parameter
        # Tank capacities (Stage 1 and 3 hold at most 400 mL)
        self.raw_cap = p('raw_tank_capacity_ml', 400.0).value
        self.clean_cap = p('clean_tank_capacity_ml', 400.0).value
        self.column_cap = p('filter_column_capacity_ml', 100.0).value
        # Flow rates
        self.intake_rate = p('intake_flow_ml_s', 20.0).value
        self.filter_rate = p('filter_flow_ml_s', 5.0).value
        self.transfer_rate = p('transfer_flow_ml_s', 8.0).value    # transfer pump column -> Stage 3
        self.delivery_rate = p('delivery_flow_ml_s', 25.0).value
        self.chlorine_rate = p('chlorine_pump_ml_s', 1.0).value
        # Dosing ratios (stock solutions, see docs/HARDWARE.md)
        self.chlorine_ml_per_l = p('chlorine_ml_per_litre', 3.0).value  # 0.07 % NaOCl stock (bleach 1:50) -> ~2 mg/L
        self.max_dose_ml = p('max_single_dose_ml', 4.0).value
        # Safety
        self.intake_max_run_s = p('intake_max_run_s', 90.0).value
        self.dry_run_s = p('dry_run_timeout_s', 3.0).value
        self.raw_full_pct = p('raw_full_pct', 95.0).value
        self.clean_full_pct = p('clean_full_pct', 95.0).value
        self.column_high_pct = p('column_high_pct', 90.0).value
        self.column_resume_pct = p('column_resume_pct', 50.0).value
        self.arm_deployed_rad = math.radians(p('arm_deployed_deg', 80.0).value)   # intake boom counts as lowered
        # Where water is reachable: rover x >= river_min_x (bank of v2_village is at x=10)
        self.river_min_x = p('river_min_x', 9.3).value
        self.chlorine_stock = p('chlorine_stock_ml', 250.0).value
        self.rate_hz = p('update_rate_hz', 10.0).value

        # ---- state ----
        self.raw_ml = 0.0
        self.column_ml = 0.0
        self.clean_ml = 0.0
        self.relay = [False, False, False, False]
        self.transfer_left_ml = 0.0
        self.chlorine_left_ml = 0.0
        self.deliver_left_ml = 0.0
        self.transfer_total_ml = 0.0
        self.chlorine_total_ml = 0.0
        self.delivered_total_ml = 0.0
        self.deliveries = 0
        self.intake_requested = False
        self.intake_run_s = 0.0
        self.dry_s = 0.0
        self.overflow_latch = False
        self.estop = False
        self.fault = ''
        self.pose_x = None
        self.arm_angle = 0.0
        self.last_event = 'boot'

        self.state_pub = self.create_publisher(String, '/waterdroid/process_state', 10)
        self.create_subscription(String, '/waterdroid/cmd', self._on_cmd, 10)
        self.create_subscription(Bool, '/waterdroid/estop', self._on_estop, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt, self._step)
        self.create_timer(0.2, self._publish)
        self.get_logger().info('Water processor (simulated treatment train) ready')

    # ------------------------------------------------------------------ inputs
    def _on_odom(self, msg):
        self.pose_x = msg.pose.pose.position.x

    def _on_joints(self, msg):
        if 'arm_servo_joint' in msg.name:
            self.arm_angle = msg.position[msg.name.index('arm_servo_joint')]

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self._all_off()
            self.fault = 'EMERGENCY_STOP'
            self.get_logger().warn('E-STOP: all pumps off')
        elif not msg.data and self.estop:
            self.fault = ''
            self.get_logger().info('E-STOP released')
        self.estop = msg.data

    def _on_cmd(self, msg):
        """Commands (case-insensitive):
            intake:start | intake:stop
            transfer[:ml]       (no ml -> move what is in the column) | transfer:stop
            dose_chlorine[:ml]  (no ml -> auto dose for the clean-tank volume)
            deliver:ml | deliver:stop
            stop | reset
        """
        text = msg.data.strip().lower()
        name, _, arg = text.partition(':')
        if name == 'reset':
            self._all_off()
            self.fault = ''
            self.overflow_latch = False
            self._event('reset')
            return
        if name == 'stop':
            self._all_off()
            self._event('all stop')
            return
        if self.estop:
            self._event(f'rejected {text}: e-stop active')
            return

        if name == 'intake':
            if arg == 'start':
                self.intake_requested = True
                self.intake_run_s = 0.0
                self.dry_s = 0.0
                if self.fault == 'DRY_RUN':
                    self.fault = ''
                self._event('intake start')
            else:
                self.intake_requested = False
                self._event('intake stop')
        elif name == 'transfer':
            if arg == 'stop':
                self.transfer_left_ml = 0.0
                self._event('transfer stop')
            else:
                try:
                    self.transfer_left_ml = max(0.0, float(arg))
                except ValueError:
                    self.transfer_left_ml = self.column_ml
                self._event(f'transfer {self.transfer_left_ml:.0f} mL column -> delivery tank')
        elif name == 'dose_chlorine':
            ml = self._dose_amount(arg, self.clean_ml, self.chlorine_ml_per_l)
            self.chlorine_left_ml = min(ml, self.chlorine_stock)
            self._event(f'chlorine dose {self.chlorine_left_ml:.2f} mL')
        elif name == 'deliver':
            if arg == 'stop':
                self.deliver_left_ml = 0.0
                self._event('delivery stop')
            else:                                   # fixed nozzle: no arm interlock
                try:
                    self.deliver_left_ml = max(0.0, float(arg))
                except ValueError:
                    self.deliver_left_ml = self.clean_ml
                self._event(f'deliver {self.deliver_left_ml:.0f} mL')
        else:
            self._event(f'unknown command {text}')

    def _dose_amount(self, arg, volume_ml, ml_per_l):
        try:
            ml = float(arg)
        except ValueError:
            ml = volume_ml / 1000.0 * ml_per_l
        return max(0.0, min(ml, self.max_dose_ml))

    # ------------------------------------------------------------------ model
    def _step(self):
        dt = self.dt
        if self.estop:
            self._all_off()
            return

        raw_pct = 100.0 * self.raw_ml / self.raw_cap
        clean_pct = 100.0 * self.clean_ml / self.clean_cap
        col_pct = 100.0 * self.column_ml / self.column_cap

        # Stage 2 overflow latch with hysteresis (same thresholds as filtration_monitor-1.ino)
        if col_pct >= self.column_high_pct:
            self.overflow_latch = True
        elif col_pct <= self.column_resume_pct:
            self.overflow_latch = False

        # ---- CH1 intake pump (Issue #1) ----
        # water reaches the pump only when the rover is at the bank AND the servo boom is lowered
        boom_down = self.arm_angle >= self.arm_deployed_rad
        in_river = self.pose_x is not None and self.pose_x >= self.river_min_x and boom_down
        want_intake = (self.intake_requested and raw_pct < self.raw_full_pct
                       and not self.overflow_latch and self.fault != 'DRY_RUN')
        # A batch is what the clean tank can hold: stop pumping once the water already
        # on board (S1 + column + S3) would fill Stage 3, otherwise S1 can never drain.
        batch_full = (self.raw_ml + self.column_ml + self.clean_ml
                      >= self.clean_cap * self.clean_full_pct / 100.0)
        if self.intake_requested and (raw_pct >= self.raw_full_pct or batch_full):
            self.intake_requested = False
            want_intake = False
            self._event('raw tank full - intake auto stop' if raw_pct >= self.raw_full_pct
                        else 'batch complete (Stage 3 capacity reached) - intake auto stop')
        if want_intake:
            self.intake_run_s += dt
            if self.intake_run_s > self.intake_max_run_s:
                want_intake = False
                self.intake_requested = False
                self._event('intake max-run cutoff (motor protection)')
            if in_river:
                self.dry_s = 0.0
                self.raw_ml = min(self.raw_cap, self.raw_ml + self.intake_rate * dt)
            else:
                self.dry_s += dt
                if self.dry_s >= self.dry_run_s:
                    want_intake = False
                    self.intake_requested = False
                    self.fault = 'DRY_RUN'
                    self._event('dry-run protection: no water at intake')
        self.relay[0] = want_intake

        # ---- Stage 2 gravity filtration S1 -> column (Issue #5) ----
        if self.raw_ml > 0 and not self.overflow_latch:
            f = min(self.filter_rate * dt, self.raw_ml, self.column_cap - self.column_ml)
            self.raw_ml -= f
            self.column_ml += f

        # ---- CH2 transfer pump: column -> Stage 3 delivery tank ----
        self.relay[1] = self.transfer_left_ml > 1e-6
        if self.relay[1]:
            d = min(self.transfer_rate * dt, self.transfer_left_ml)
            self.transfer_left_ml -= d
            self.transfer_total_ml += d
            moved = min(d, self.column_ml, self.clean_cap - self.clean_ml)   # pump runs dry if the estimate was high
            self.column_ml -= moved
            self.clean_ml += moved

        # ---- CH3 chlorine dosing into Stage 3 (Issue #3) ----
        self.relay[2] = self.chlorine_left_ml > 1e-6
        if self.relay[2]:
            d = min(self.chlorine_rate * dt, self.chlorine_left_ml)
            self.chlorine_left_ml -= d
            self.chlorine_total_ml += d
            self.chlorine_stock -= d
            self.clean_ml = min(self.clean_cap, self.clean_ml + d)

        # ---- CH4 delivery pump Stage 3 -> container (Issue #8) ----
        self.relay[3] = self.deliver_left_ml > 1e-6 and self.clean_ml > 1.0
        if self.deliver_left_ml > 1e-6 and self.clean_ml <= 1.0:
            self.deliver_left_ml = 0.0
            self.deliveries += 1
            self._event(f'delivery #{self.deliveries} ended early: clean tank empty')
        if self.relay[3]:
            d = min(self.delivery_rate * dt, self.deliver_left_ml, self.clean_ml)
            self.deliver_left_ml -= d
            self.clean_ml -= d
            self.delivered_total_ml += d
            if self.deliver_left_ml <= 1e-6:
                self.deliveries += 1
                self._event(f'delivery #{self.deliveries} complete')

    def _all_off(self):
        self.intake_requested = False
        self.transfer_left_ml = self.chlorine_left_ml = self.deliver_left_ml = 0.0
        self.relay = [False, False, False, False]

    def _health(self):
        if self.estop:
            return 'MANUAL_PAUSE'
        if self.fault == 'DRY_RUN':
            return 'SOURCE_LOW'
        if self.overflow_latch:
            return 'OVERFLOW'
        if 100.0 * self.clean_ml / self.clean_cap >= self.clean_full_pct:
            return 'TANK_FULL'
        return 'HEALTHY'

    def _event(self, text):
        self.last_event = text
        self.get_logger().info(text)

    # ------------------------------------------------------------------ output
    def _publish(self):
        busy = any(self.relay) or self.intake_requested
        clean_full = self.clean_ml >= self.clean_cap - 1.0
        filtering = (self.raw_ml > 1.0 or self.column_ml > 1.0) and not clean_full
        state = {
            's1_ml': round(self.raw_ml, 1),
            's2_ml': round(self.column_ml, 1),
            's3_ml': round(self.clean_ml, 1),
            's1': round(100.0 * self.raw_ml / self.raw_cap, 1),
            's2': round(100.0 * self.column_ml / self.column_cap, 1),
            's3': round(100.0 * self.clean_ml / self.clean_cap, 1),
            'relays': dict(zip(RELAY_NAMES, self.relay)),
            'intake': self.relay[0],
            'intake_requested': self.intake_requested,
            'busy': busy,
            'filtering': filtering,
            'transfer_total_ml': round(self.transfer_total_ml, 1),
            'chlorine_total_ml': round(self.chlorine_total_ml, 2),
            'chlorine_stock_ml': round(self.chlorine_stock, 1),
            'delivered_total_ml': round(self.delivered_total_ml, 1),
            'deliveries': self.deliveries,
            'arm_deg': round(math.degrees(self.arm_angle), 1),
            'health': self._health(),
            'fault': self.fault,
            'estop': self.estop,
            'event': self.last_event,
        }
        self.state_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = WaterProcessor()
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
