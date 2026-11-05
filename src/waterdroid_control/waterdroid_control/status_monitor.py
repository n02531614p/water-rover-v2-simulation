#!/usr/bin/env python3
"""Status monitor + Node-RED/MQTT bridge for the simulated rover (Issue #12).

Aggregates the mission, treatment-train, arm and pose state into one telemetry
frame and (optionally) mirrors it onto the SAME MQTT topics the real ESP32 uses,
so the Node-RED dashboard in node-red/ works unchanged against Gazebo.

  ROS  pub /waterdroid/status        std_msgs/String JSON, 1 Hz
       pub /waterdroid/estop         std_msgs/Bool   (from rover/estop or power emergency)
       pub /waterdroid/mission/cmd   std_msgs/String
       pub /waterdroid/cmd, /waterdroid/arm/cmd (from dashboard buttons)
       pub /cmd_vel                  teleop, only while the mission is paused/idle/complete

  MQTT (when mqtt_enabled:=true and paho-mqtt is installed, see docs/TOPICS.md)
       out rover/telemetry, rover/state, rover/nav/pose, rover/intake/status,
           rover/dosing/status, rover/transfer/status, rover/filtration/status, rover/arm/status,
           rover/delivery/status, rover/drive/status, rover/power/status
       in  rover/estop, rover/mission/cmd, rover/intake/cmd, rover/dosing/cmd, rover/transfer/cmd,
           rover/filtration/cmd, rover/arm/cmd, rover/delivery/cmd, rover/drive/cmd,
           rover/power/cmd
"""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import paho.mqtt.client as paho
except ImportError:          # bridge is optional
    paho = None

# Simulated current draw (mA) used for the battery estimate - see docs/HARDWARE.md
LOAD_MA = {'base': 180, 'drive': 900, 'intake_pump': 350, 'transfer_pump': 350,
           'chlorine_pump': 150, 'delivery_pump': 350, 'servo': 150}
TELEOP_STATES = ('STATE_IDLE', 'STATE_PAUSED', 'STATE_COMPLETE')


class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        p = self.declare_parameter
        self.mqtt_enabled = p('mqtt_enabled', False).value
        self.mqtt_host = p('mqtt_host', 'localhost').value
        self.mqtt_port = int(p('mqtt_port', 1883).value)
        self.capacity_mah = p('battery_capacity_mah', 10000.0).value
        self.usable = p('battery_usable_fraction', 0.85).value
        self.teleop_lin = p('teleop_linear', 0.3).value
        self.teleop_ang = p('teleop_angular', 1.5).value

        self.mission = {'state': 'STATE_IDLE', 'state_id': 0}
        self.proc = {}
        self.arm = {}
        self.pose = (0.0, 0.0, 0.0)
        self.speed = 0.0
        self.used_mah = 0.0
        self.estop = False
        self.power_mode = 'ACTIVE'
        self.teleop = None
        self.teleop_until = 0.0
        self.last_t = time.monotonic()

        self.status_pub = self.create_publisher(String, '/waterdroid/status', 10)
        self.estop_pub = self.create_publisher(Bool, '/waterdroid/estop', 10)
        self.mission_pub = self.create_publisher(String, '/waterdroid/mission/cmd', 10)
        self.proc_pub = self.create_publisher(String, '/waterdroid/cmd', 10)
        self.arm_pub = self.create_publisher(String, '/waterdroid/arm/cmd', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, '/waterdroid/mission_state', self._json_into('mission'), 10)
        self.create_subscription(String, '/waterdroid/process_state', self._json_into('proc'), 10)
        self.create_subscription(String, '/waterdroid/arm/state', self._json_into('arm'), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Bool, '/waterdroid/estop', self._on_estop, 10)

        self.create_timer(1.0, self._tick)
        self.create_timer(0.05, self._teleop_tick)

        self.mqtt = None
        if self.mqtt_enabled:
            self._start_mqtt()
        self.get_logger().info('Status monitor ready' + (' (MQTT bridge on)' if self.mqtt else ''))

    # ------------------------------------------------------------ ROS inputs
    def _json_into(self, attr):
        def cb(msg):
            try:
                setattr(self, attr, json.loads(msg.data))
            except ValueError:
                pass
        return cb

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_cmd_vel(self, msg):
        self.speed = abs(msg.linear.x) + 0.1 * abs(msg.angular.z)

    def _on_estop(self, msg):
        self.estop = msg.data

    # ------------------------------------------------------------ battery model
    def _current_ma(self):
        relays = self.proc.get('relays', {})
        ma = LOAD_MA['base']
        if self.speed > 0.01:
            ma += LOAD_MA['drive']
        for name, on in relays.items():
            if on:
                ma += LOAD_MA.get(name, 0)
        if self.arm.get('moving'):
            ma += LOAD_MA['servo']
        return ma

    def _power_frame(self, ma):
        left = max(0.0, 100.0 * (1.0 - self.used_mah / (self.capacity_mah * self.usable)))
        rail_v = round(5.08 - 0.00012 * ma - (0.25 if left < 5 else 0.0), 2)
        relays = self.proc.get('relays', {})
        ch = [int(bool(relays.get(k))) for k in
              ('intake_pump', 'transfer_pump', 'chlorine_pump', 'delivery_pump')]
        mode = 'EMERGENCY' if self.estop else self.power_mode
        return {
            'mode': mode, 'mode_id': {'ACTIVE': 0, 'SLEEP': 1, 'EMERGENCY': 2}[mode],
            'rail_v': rail_v, 'battery_v': rail_v,
            'rail': 'OK' if rail_v >= 4.5 else ('SAG' if rail_v >= 4.2 else 'CRITICAL'),
            'ch': ch, 'want': ch, 'pending': 0, 'brownouts': 0,
            'fault': 'operator emergency shutdown' if self.estop else '',
            'uptime_s': int(self.get_clock().now().nanoseconds / 1e9),
            'current_ma': ma, 'charge_pct': round(left, 1),
        }

    # ------------------------------------------------------------ 1 Hz frame
    def _tick(self):
        now = time.monotonic()
        dt, self.last_t = now - self.last_t, now
        ma = self._current_ma()
        self.used_mah += ma * dt / 3600.0
        power = self._power_frame(ma)

        x, y, yaw = self.pose
        frame = {
            'state': self.mission.get('state'),
            'state_id': self.mission.get('state_id', 0),
            'bay': self.mission.get('bay'),
            'pose': {'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2),
                     'target': self.mission.get('target')},
            'speed': round(self.speed, 2),
            'battery_v': power['rail_v'],
            'charge_pct': power['charge_pct'],
            'process': self.proc,
            'arm': self.arm,
            'estop': self.estop,
        }
        self.status_pub.publish(String(data=json.dumps(frame)))
        if self.mqtt:
            self._mqtt_publish_all(frame, power)

    # ------------------------------------------------------------ MQTT bridge
    def _start_mqtt(self):
        if paho is None:
            self.get_logger().warn('mqtt_enabled but paho-mqtt is missing: pip install paho-mqtt')
            return
        try:
            client = paho.Client(paho.CallbackAPIVersion.VERSION2, client_id='waterdroid-sim')
        except AttributeError:      # paho-mqtt < 2.0
            client = paho.Client(client_id='waterdroid-sim')
        client.on_connect = self._mqtt_on_connect
        client.on_message = self._mqtt_on_message
        client.will_set('rover/online', '0', retain=True)
        try:
            client.connect_async(self.mqtt_host, self.mqtt_port, 30)
            client.loop_start()
            self.mqtt = client
        except OSError as e:
            self.get_logger().error(f'MQTT connect failed: {e}')

    def _mqtt_on_connect(self, client, *_args):
        for t in ('rover/estop', 'rover/mission/cmd', 'rover/intake/cmd', 'rover/dosing/cmd', 'rover/transfer/cmd',
                  'rover/filtration/cmd', 'rover/arm/cmd', 'rover/delivery/cmd',
                  'rover/drive/cmd', 'rover/power/cmd'):
            client.subscribe(t)
        client.publish('rover/online', '1', retain=True)
        self.get_logger().info(f'MQTT connected to {self.mqtt_host}:{self.mqtt_port}')

    def _pub(self, topic, obj):
        self.mqtt.publish(topic, json.dumps(obj) if not isinstance(obj, str) else obj)

    def _mqtt_publish_all(self, frame, power):
        pr = self.proc
        self._pub('rover/telemetry', frame)
        self._pub('rover/state', {'state': frame['state'], 'state_id': frame['state_id'],
                                  'battery_v': frame['battery_v']})
        self._pub('rover/nav/pose', frame['pose'])
        self._pub('rover/intake/status', {
            'pump': pr.get('intake', False), 'level_pct': pr.get('s1', 0),
            'fault': pr.get('fault', ''), 'run_s': None})
        self._pub('rover/dosing/status', {
            'chlorine_on': pr.get('relays', {}).get('chlorine_pump', False),
            'chlorine_total_ml': pr.get('chlorine_total_ml', 0), 'chlorine_stock_ml': pr.get('chlorine_stock_ml')})
        self._pub('rover/transfer/status', {
            'pump': pr.get('relays', {}).get('transfer_pump', False), 'total_ml': pr.get('transfer_total_ml', 0),
            'column_ml': pr.get('s2_ml', 0), 'clean_ml': pr.get('s3_ml', 0)})
        self._pub('rover/filtration/status', {
            's1': pr.get('s1', 0), 's2': pr.get('s2', 0), 's3': pr.get('s3', 0),
            'intake': pr.get('intake', False), 'flow': 0.0, 'health': pr.get('health', 'HEALTHY')})
        self._pub('rover/arm/status', self.arm)
        self._pub('rover/delivery/status', {
            'pump': pr.get('relays', {}).get('delivery_pump', False),
            'delivered_total_ml': pr.get('delivered_total_ml', 0),
            'deliveries': pr.get('deliveries', 0), 'clean_ml': pr.get('s3_ml', 0)})
        self._pub('rover/drive/status', {'speed': frame['speed'], 'teleop': self.teleop or 'none'})
        self._pub('rover/power/status', power)

    def _mqtt_on_message(self, _client, _userdata, msg):
        topic = msg.topic
        text = msg.payload.decode(errors='ignore').strip()
        try:
            data = json.loads(text)
        except ValueError:
            data = text
        if isinstance(data, (int, float)) and not isinstance(data, bool):
            data = str(data)
        low = data.lower() if isinstance(data, str) else data

        if topic == 'rover/estop':
            self.estop_pub.publish(Bool(data=low in ('1', 'true', 'on', 'estop', True)))
        elif topic == 'rover/power/cmd':
            if low in ('emergency', 'estop'):
                self.estop_pub.publish(Bool(data=True))
            elif low == 'reset':
                self.estop_pub.publish(Bool(data=False))
            elif low in ('active', 'sleep'):
                self.power_mode = low.upper()
                if low == 'sleep':
                    self.proc_pub.publish(String(data='stop'))
        elif topic == 'rover/mission/cmd':
            self.mission_pub.publish(String(data=str(low)))
        elif topic == 'rover/intake/cmd':
            if low in ('start', 'on', '1'):
                self.arm_pub.publish(String(data='EXTEND'))      # lower the servo intake first
                self.proc_pub.publish(String(data='intake:start'))
            else:
                self.proc_pub.publish(String(data='intake:stop'))
        elif topic == 'rover/filtration/cmd':
            if low == 'pause':
                self.proc_pub.publish(String(data='intake:stop'))
        elif topic == 'rover/dosing/cmd':
            if isinstance(data, dict) and str(data.get('pump', '')).startswith('chlor'):
                ml = data.get('ml')
                self.proc_pub.publish(String(data=f'dose_chlorine:{ml}' if ml is not None else 'dose_chlorine'))
            elif low == 'stop':
                self.proc_pub.publish(String(data='stop'))
        elif topic == 'rover/transfer/cmd':
            if isinstance(data, dict) and 'ml' in data:
                self.proc_pub.publish(String(data=f"transfer:{data['ml']}"))
            else:
                self.proc_pub.publish(String(data='transfer:stop' if low == 'stop' else 'transfer'))
        elif topic == 'rover/arm/cmd':
            self.arm_pub.publish(String(data=str(data)))
        elif topic == 'rover/delivery/cmd':
            if isinstance(data, dict) and 'ml' in data:
                self.proc_pub.publish(String(data=f"deliver:{data['ml']}"))
            else:
                self.proc_pub.publish(String(data='deliver:stop'))
        elif topic == 'rover/drive/cmd':
            self._teleop_cmd(low)

    # ------------------------------------------------------------ teleop (Issue #9)
    def _teleop_cmd(self, cmd):
        if self.mission.get('state') not in TELEOP_STATES:
            self.get_logger().warn('Teleop ignored: pause the mission first')
            return
        self.teleop = cmd if cmd in ('forward', 'reverse', 'left', 'right') else None
        self.teleop_until = time.monotonic() + 0.6     # dead-man: buttons must repeat

    def _teleop_tick(self):
        if self.teleop is None:
            return
        if self.estop or time.monotonic() > self.teleop_until:
            self.teleop = None
            self.cmd_pub.publish(Twist())
            return
        t = Twist()
        t.linear.x = {'forward': self.teleop_lin, 'reverse': -self.teleop_lin}.get(self.teleop, 0.0)
        t.angular.z = {'left': self.teleop_ang, 'right': -self.teleop_ang}.get(self.teleop, 0.0)
        self.cmd_pub.publish(t)

    def destroy_node(self):
        if self.mqtt:
            self.mqtt.publish('rover/online', '0', retain=True)
            self.mqtt.loop_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
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
