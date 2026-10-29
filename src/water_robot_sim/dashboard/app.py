#!/usr/bin/env python3
import threading
import time
from flask import Flask, render_template, jsonify, request
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

app = Flask(__name__)

# Global Telemetry State
telemetry = {
    "status": "DOCKED / IDLE",
    "pos_x": 0.0,
    "pos_y": 0.0,
    "speed": 0.0,
    "battery": 98,
    "water_level": 0,       # mL
    "turbidity": 45.2,      # NTU
    "boil_temp": 24.5,      # Celsius
    "boil_status": "OFF",
    "batch_count": 1,
    "batch_history": [
        {"id": "BATCH-001", "time": "10:14 AM", "volume": "350 mL", "turbidity_in": "45 NTU", "turbidity_out": "12 NTU", "status": "PASSED"}
    ]
}

class DashboardRosNode(Node):
    def __init__(self):
        super().__init__('dashboard_bridge_node')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

    def odom_cb(self, msg):
        telemetry["pos_x"] = round(msg.pose.pose.position.x, 2)
        telemetry["pos_y"] = round(msg.pose.pose.position.y, 2)

    def cmd_cb(self, msg):
        speed = abs(msg.linear.x) + abs(msg.angular.z)
        telemetry["speed"] = round(speed, 2)
        
        # Dynamic telemetry updates based on position
        if speed > 0.05:
            telemetry["status"] = "NAVIGATING"
        else:
            if telemetry["pos_x"] > 5.0:
                telemetry["status"] = "PUMPING WATER"
                telemetry["water_level"] = min(350, telemetry["water_level"] + 10)
            elif telemetry["pos_x"] < 0.5 and telemetry["water_level"] > 0:
                telemetry["status"] = "FILTERING & BOILING"
                telemetry["boil_status"] = "HEATING"
                if telemetry["boil_temp"] < 100.0:
                    telemetry["boil_temp"] = round(telemetry["boil_temp"] + 2.5, 1)
                else:
                    telemetry["boil_status"] = "ROLLING BOIL (100 C)"
                    telemetry["turbidity"] = 12.1

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/telemetry')
def get_telemetry():
    return jsonify(telemetry)

def run_ros():
    rclpy.init()
    node = DashboardRosNode()
    rclpy.spin(node)

if __name__ == '__main__':
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    app.run(host='0.0.0.0', port=5000)
