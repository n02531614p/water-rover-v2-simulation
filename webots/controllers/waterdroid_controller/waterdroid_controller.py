from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
if gps:
    gps.enable(timestep)

compass = robot.getDevice("compass")
if compass:
    compass.enable(timestep)

fl_motor = robot.getDevice("front_left_motor")
rl_motor = robot.getDevice("left wheel motor")
fr_motor = robot.getDevice("front_right_motor")
rr_motor = robot.getDevice("right wheel motor")

motors = [fl_motor, rl_motor, fr_motor, rr_motor]

for m in motors:
    if m:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

waypoints = [
    {"name": "RIVER_INTAKE", "x": 10.0, "y": 0.0, "wait": 3.0, "action": "Collecting Raw Water"},
    {"name": "HOME_1", "x": -5.0, "y": 3.0, "wait": 3.0, "action": "Offloading to Home 1"},
    {"name": "HOME_2", "x": -5.0, "y": -0.5, "wait": 3.0, "action": "Offloading to Home 2"},
    {"name": "HOME_3", "x": -5.0, "y": -3.0, "wait": 3.0, "action": "Offloading to Home 3"},
    {"name": "RETURN_PLANT", "x": 0.0, "y": 0.0, "wait": 2.0, "action": "Docked at Plant Base"}
]

wp_idx = 0
wait_counter = 0

def set_4wd_speed(left_speed, right_speed):
    if fl_motor: fl_motor.setVelocity(left_speed)
    if rl_motor: rl_motor.setVelocity(left_speed)
    if fr_motor: fr_motor.setVelocity(right_speed)
    if rr_motor: rr_motor.setVelocity(right_speed)

def get_bearing():
    if not compass:
        return 0.0
    north = compass.getValues()
    rad = math.atan2(north[0], north[1])
    return rad

print("--- Compass-Guided 4WD Waterdroid Controller Active ---")

while robot.step(timestep) != -1:
    if wp_idx >= len(waypoints):
        set_4wd_speed(0.0, 0.0)
        print("MISSION COMPLETE: All homesteads served successfully!")
        break

    target = waypoints[wp_idx]
    
    pos = gps.getValues() if gps else [0.0, 0.0, 0.0]
    curr_x, curr_y = pos[0], pos[1]

    dx = target["x"] - curr_x
    dy = target["y"] - curr_y
    dist = math.sqrt(dx * dx + dy * dy)

    # Orientation via Compass
    current_heading = get_bearing()
    desired_heading = math.atan2(dy, dx)

    angle_error = desired_heading - current_heading
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    if dist > 0.4:
        if abs(angle_error) > 0.4:
            # In-place turn towards waypoint
            turn_speed = 3.0 if angle_error > 0 else -3.0
            set_4wd_speed(-turn_speed, turn_speed)
        else:
            # Drive forward with slight heading correction
            base_speed = 6.0
            correction = angle_error * 3.0
            set_4wd_speed(base_speed - correction, base_speed + correction)
    else:
        set_4wd_speed(0.0, 0.0)

        if wait_counter == 0:
            print(f"ARRIVED AT {target['name']}: {target['action']}")

        wait_counter += timestep / 1000.0

        if wait_counter >= target["wait"]:
            print(f"COMPLETED {target['name']}. Moving to next destination...")
            wp_idx += 1
            wait_counter = 0
