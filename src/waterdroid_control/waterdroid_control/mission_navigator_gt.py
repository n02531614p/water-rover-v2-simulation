#!/usr/bin/env python3
"""Waterdroid delivery mission on ground-truth odometry (Issue #11 state machine).

Why ground truth: the 4WD skid-steer DiffDrive odometry reports motion the body
did not make when the wheels slip, so the mission uses the pose published by the
Gazebo OdometryPublisher (/ground_truth/odom). The first pose is the origin, so the
route below is relative to the spawn pose (v2_village: depot at 0,0, river bank at
x=10, offload bays at (6,3), (6,-3) and (-3,-3)).

Mission (state names are the Master System State shown on the Node-RED dashboard):
  STATE_EN_ROUTE_RIVER      drive east to the river bank
  STATE_PUMPING             servo lowers the intake, pump fills Stage 1 + alum, intake raised
  STATE_FILTERING           wait for the gravity column to pass the batch to Stage 3
  STATE_DOSING_CHLORINE     chlorine into Stage 3 + contact time
  STATE_EN_ROUTE_HOMESTEAD  reverse off the bank, pivot, drive to each bay
  STATE_DISPENSING          delivery pump through the fixed nozzle
  STATE_RETURNING           back to the depot
  STATE_COMPLETE / STATE_PAUSED / STATE_EMERGENCY_STOP

  sub /ground_truth/odom, /waterdroid/process_state, /waterdroid/arm/state,
      /waterdroid/estop (Bool), /waterdroid/mission/cmd (start|pause|resume|abort)
  pub /cmd_vel, /waterdroid/cmd, /waterdroid/arm/cmd, /waterdroid/mission_state
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

BAYS = [
    ('HOMESTEAD_1', 6.0, 3.0),
    ('HOMESTEAD_2', 6.0, -3.0),
    ('HOMESTEAD_3', -3.0, -3.5),
]
STATE_IDS = {
    'STATE_IDLE': 0, 'STATE_EN_ROUTE_RIVER': 1, 'STATE_PUMPING': 2, 'STATE_DOSING_ALUM': 2,
    'STATE_FILTERING': 2, 'STATE_DOSING_CHLORINE': 2, 'STATE_EN_ROUTE_HOMESTEAD': 3,
    'STATE_DISPENSING': 4, 'STATE_RETURNING': 3, 'STATE_COMPLETE': 0,
    'STATE_PAUSED': 0, 'STATE_EMERGENCY_STOP': 5,
}


class MissionNavigator(Node):
    def __init__(self):
        super().__init__('mission_navigator')

        p = self.declare_parameter
        self.autostart = p('autostart', True).value
        self.bank_x = p('river_stop_x', 9.6).value
        self.drive_speed = p('drive_speed', 0.35).value
        self.arrive_tol = p('arrive_tolerance', 0.30).value
        self.pump_timeout = p('pump_timeout_s', 120.0).value
        self.filter_timeout = p('filter_timeout_s', 300.0).value
        self.contact_s = p('chlorine_contact_s', 5.0).value
        self.alum_ml_per_l = p('alum_ml_per_litre', 3.0).value
        self.raw_cap_ml = p('raw_tank_capacity_ml', 380.0).value   # batch per trip
        self.action_timeout = p('action_timeout_s', 60.0).value
        # 'ground_truth' = perfect pose from Gazebo; 'dead_reckoning' = like the real rover
        # (no GPS/compass): pose integrated from the commanded /cmd_vel only, so wheel slip
        # shows up as drift. The true pose is still logged at every stop for comparison.
        self.pose_source = p('pose_source', 'ground_truth').value
        # Dead-reckoning calibration, the sim twin of NAV_SPEED_MPS / NAV_TURN_DPS on the ESP32:
        # measured speed / commanded speed (run calib, measure, divide). Skid steer slips most when pivoting.
        self.dr_speed_scale = p('dr_speed_scale', 1.0).value
        self.dr_turn_scale = p('dr_turn_scale', 1.0).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.proc_pub = self.create_publisher(String, '/waterdroid/cmd', 10)
        self.arm_pub = self.create_publisher(String, '/waterdroid/arm/cmd', 10)
        self.state_pub = self.create_publisher(String, '/waterdroid/mission_state', 10)

        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)
        self.create_subscription(String, '/waterdroid/process_state', self._on_proc, 10)
        self.create_subscription(String, '/waterdroid/arm/state', self._on_arm, 10)
        self.create_subscription(Bool, '/waterdroid/estop', self._on_estop, 10)
        self.create_subscription(String, '/waterdroid/mission/cmd', self._on_mission_cmd, 10)

        self.pose_x = self.pose_y = self.yaw = 0.0
        self.origin_x = self.origin_y = self.origin_yaw = None
        self.proc = {}
        self.arm = {}
        self.estop = False
        self.paused = not self.autostart

        self.phase = 'DRIVE_RIVER'
        self.sub = 0                # sub-step inside a phase
        self.t0 = None              # phase timer (sim time)
        self.bay_idx = 0
        self.mission_state = 'STATE_IDLE'

        self.true_x = self.true_y = self.true_yaw = 0.0
        self.last_twist = Twist()
        self.dr_last = None
        if self.pose_source == 'dead_reckoning':
            self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(0.02, self.control_loop)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info('=== Waterdroid 4WD mission (ground-truth odom) active ===')

    # ------------------------------------------------------------ callbacks
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.origin_x is None:
            self.origin_x, self.origin_y, self.origin_yaw = x, y, yaw
            self.get_logger().info(f'Ground-truth origin set: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        self.true_x = x - self.origin_x
        self.true_y = y - self.origin_y
        self.true_yaw = self._wrap(yaw - self.origin_yaw)
        if self.pose_source != 'dead_reckoning':
            self.pose_x, self.pose_y, self.yaw = self.true_x, self.true_y, self.true_yaw

    def _on_cmd_vel(self, msg):
        self.last_twist = msg

    def _dead_reckon(self):
        now = self.now()
        if self.dr_last is not None:
            dt = now - self.dr_last
            v = self.last_twist.linear.x * self.dr_speed_scale
            w = self.last_twist.angular.z * self.dr_turn_scale
            self.yaw = self._wrap(self.yaw + w * dt)
            self.pose_x += v * math.cos(self.yaw) * dt
            self.pose_y += v * math.sin(self.yaw) * dt
        self.dr_last = now

    def _drift_note(self):
        if self.pose_source != 'dead_reckoning':
            return ''
        err = math.hypot(self.pose_x - self.true_x, self.pose_y - self.true_y)
        return (f' | dead-reckoned ({self.pose_x:.2f},{self.pose_y:.2f}) true ({self.true_x:.2f},'
                f'{self.true_y:.2f}) drift {err:.2f} m')

    def _on_proc(self, msg):
        try:
            self.proc = json.loads(msg.data)
        except ValueError:
            pass

    def _on_arm(self, msg):
        try:
            self.arm = json.loads(msg.data)
        except ValueError:
            pass

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('E-STOP received: halting mission')
            self.proc_pub.publish(String(data='stop'))
        self.estop = msg.data

    def _on_mission_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('start', 'resume'):
            if self.phase == 'COMPLETE':
                self.phase, self.sub, self.bay_idx = 'DRIVE_RIVER', 0, 0
            self.paused = False
        elif cmd == 'pause':
            self.paused = True
            self.proc_pub.publish(String(data='stop'))
        elif cmd == 'abort':
            self.paused = False
            self.proc_pub.publish(String(data='stop'))
            self.arm_pub.publish(String(data='RETRACT'))
            self._goto('RETURN')
        self.get_logger().info(f'mission command: {cmd}')

    # ------------------------------------------------------------ helpers
    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def stop(self):
        self.cmd_pub.publish(Twist())
        self._halted = False

    def stop_once(self):
        """Publish one zero command, then stay silent so dashboard teleop can drive."""
        if not getattr(self, '_halted', False):
            self.cmd_pub.publish(Twist())
            self._halted = True

    def _set_state(self, name):
        if name != self.mission_state:
            self.mission_state = name
            self.get_logger().info(f'>>> {name}')
            self._publish_state()

    def _publish_state(self):
        self.state_pub.publish(String(data=json.dumps({
            'state': self.mission_state,
            'state_id': STATE_IDS.get(self.mission_state, 0),
            'phase': self.phase,
            'bay': (BAYS[self.bay_idx][0] if self.phase in ('DRIVE_BAY', 'DISPENSE')
                    and self.bay_idx < len(BAYS) else None),
            'x': round(self.pose_x, 2), 'y': round(self.pose_y, 2), 'yaw': round(self.yaw, 2),
            'target': self._target(),
            'pose_source': self.pose_source,
            'true': [round(self.true_x, 2), round(self.true_y, 2)],
        })))

    def _target(self):
        if self.phase == 'DRIVE_RIVER':
            return [self.bank_x, 0.0]
        if self.phase in ('DRIVE_BAY', 'DISPENSE') and self.bay_idx < len(BAYS):
            return [BAYS[self.bay_idx][1], BAYS[self.bay_idx][2]]
        if self.phase == 'RETURN':
            return [0.0, 0.0]
        return None

    def _goto(self, phase):
        self.phase, self.sub, self.t0 = phase, 0, self.now()

    def _elapsed(self):
        return self.now() - (self.t0 if self.t0 is not None else self.now())

    def drive_towards(self, tx, ty, speed=None):
        speed = speed or self.drive_speed
        dx, dy = tx - self.pose_x, ty - self.pose_y
        dist = math.hypot(dx, dy)
        err = self._wrap(math.atan2(dy, dx) - self.yaw)
        cmd = Twist()
        if self.pose_source == 'dead_reckoning':
            # Same as the ESP32 RouteNavigator: pivot at ONE fixed rate, then drive dead straight,
            # so the single calibrated pivot rate stays valid (blended steering slips differently).
            if abs(err) > 0.12 and dist > 0.05:
                cmd.angular.z = 0.6 if err > 0 else -0.6
            else:
                cmd.linear.x = speed
        elif abs(err) > 0.35:
            cmd.angular.z = 1.0 if err > 0 else -1.0
        else:
            cmd.linear.x = min(speed, max(0.15, 0.5 * dist))
            cmd.angular.z = 1.0 * err
        self.cmd_pub.publish(cmd)
        return dist

    def _proc_idle(self, key):
        return not self.proc.get('relays', {}).get(key, False)

    # ------------------------------------------------------------ main loop
    def control_loop(self):
        if self.origin_x is None:
            self.stop()
            return
        if self.pose_source == 'dead_reckoning':
            self._dead_reckon()          # also while paused, so dashboard teleop is tracked
        if self.estop:
            self.stop()
            self._set_state('STATE_EMERGENCY_STOP')
            return
        if self.paused:
            self.stop_once()
            self._set_state('STATE_PAUSED' if self.mission_state != 'STATE_IDLE' else 'STATE_IDLE')
            return
        if self.t0 is None:
            self.t0 = self.now()

        getattr(self, '_phase_' + self.phase.lower())()

    # 1. drive to the river bank (front bumper near the bank, not over it)
    def _phase_drive_river(self):
        self._set_state('STATE_EN_ROUTE_RIVER')
        if self.pose_x < self.bank_x:
            cmd = Twist()
            cmd.linear.x = self.drive_speed
            if self.pose_source != 'dead_reckoning':
                cmd.angular.z = -0.8 * self.pose_y    # hold the road centre line
            self.cmd_pub.publish(cmd)
        else:
            self.stop()
            self.get_logger().info(f'Arrived at river bank x={self.pose_x:.2f}' + self._drift_note())
            self._goto('PUMP')

    # 2. lower the servo intake boom, pump + in-line alum dose, raise the boom (never while driving)
    def _phase_pump(self):
        self.stop()
        self._set_state('STATE_PUMPING')
        timed_out = self._elapsed() > self.action_timeout
        if self.sub == 0:
            self.arm_pub.publish(String(data='EXTEND'))
            self.sub, self.t0 = 1, self.now()
        elif self.sub == 1 and (self.arm.get('deployed') or timed_out):
            alum_ml = self.raw_cap_ml / 1000.0 * self.alum_ml_per_l
            self.proc_pub.publish(String(data='intake:start'))
            self.proc_pub.publish(String(data=f'dose_alum:{alum_ml:.2f}'))
            self.sub, self.t0 = 2, self.now()
        elif self.sub == 2 and self._elapsed() > 1.0:
            done = not self.proc.get('intake', False) and self._proc_idle('alum_pump')
            if done or self._elapsed() > self.pump_timeout:
                if self.proc.get('fault'):
                    self.get_logger().warn(f"intake ended with fault {self.proc.get('fault')}")
                self.arm_pub.publish(String(data='RETRACT'))
                self.sub, self.t0 = 3, self.now()
        elif self.sub == 3 and (self.arm.get('stowed') or timed_out):
            self._goto('FILTER')

    # 3. wait for the gravity column to pass the batch to Stage 3
    def _phase_filter(self):
        self.stop()
        self._set_state('STATE_FILTERING')
        if self._elapsed() > 1.0 and (not self.proc.get('filtering', True)
                                      or self._elapsed() > self.filter_timeout):
            self._goto('CHLORINATE')

    # 4. chlorine into Stage 3 + contact time
    def _phase_chlorinate(self):
        self.stop()
        self._set_state('STATE_DOSING_CHLORINE')
        if self.sub == 0:
            self.proc_pub.publish(String(data='dose_chlorine'))
            self.sub, self.t0 = 1, self.now()
        elif self.sub == 1 and self._elapsed() > 1.0 and self._proc_idle('chlorine_pump'):
            self.sub, self.t0 = 2, self.now()
        elif self.sub == 2 and self._elapsed() >= self.contact_s:
            self._goto('REVERSE')

    # 5. reverse off the bank so all four wheels are on flat ground for the pivot
    def _phase_reverse(self):
        self._set_state('STATE_EN_ROUTE_HOMESTEAD')
        if self._elapsed() < 2.5:
            cmd = Twist()
            cmd.linear.x = -0.25
            self.cmd_pub.publish(cmd)
        else:
            self.stop()
            self._goto('PIVOT')

    # 6. closed-loop 180 degree pivot (face west, yaw = pi)
    def _phase_pivot(self):
        if self._elapsed() < 0.5:
            self.stop()
            return
        err = self._wrap(math.pi - self.yaw)
        if abs(err) > 0.15:
            cmd = Twist()
            cmd.angular.z = 0.6 if err > 0 else -0.6
            self.cmd_pub.publish(cmd)
        else:
            self.stop()
            self.bay_idx = 0
            self._goto('DRIVE_BAY')

    # 7. drive to the current offload bay
    def _phase_drive_bay(self):
        self._set_state('STATE_EN_ROUTE_HOMESTEAD')
        name, x, y = BAYS[self.bay_idx]
        if self.drive_towards(x, y) < self.arrive_tol + 0.15:
            self.stop()
            self.get_logger().info(f'Arrived at {name}' + self._drift_note())
            self._goto('DISPENSE')

    # 8. delivery pump through the fixed nozzle
    def _phase_dispense(self):
        self.stop()
        self._set_state('STATE_DISPENSING')
        timed_out = self._elapsed() > self.action_timeout
        if self.sub == 0:
            remaining = len(BAYS) - self.bay_idx
            ml = self.proc.get('s3_ml', 0.0) / max(1, remaining)
            self.proc_pub.publish(String(data=f'deliver:{int(ml)}'))
            self.sub, self.t0 = 1, self.now()
        elif self.sub == 1 and self._elapsed() > 1.0 and (self._proc_idle('delivery_pump') or timed_out):
            self.bay_idx += 1
            self._goto('DRIVE_BAY' if self.bay_idx < len(BAYS) else 'RETURN')

    # 9. back to the depot
    def _phase_return(self):
        self._set_state('STATE_RETURNING')
        if self.drive_towards(0.0, 0.0) < 0.4:
            self.stop()
            self.get_logger().info('=== MISSION COMPLETE: Waterdroid parked at depot ===' + self._drift_note())
            self._goto('COMPLETE')

    def _phase_complete(self):
        self.stop_once()
        self._set_state('STATE_COMPLETE')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNavigator()
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
