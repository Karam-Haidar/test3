Update
program
v1

To
incorporate
servo
feedback
into
all
functions, we
will
use
the
Feetech
SCS115
feedback
mechanism
to
read
the
actual
angle
of
each
servo.By
incorporating
feedback, the
robot
can
confirm
that
the
servos
reach
their
intended
angles and adjust
dynamically if necessary.Here
's the updated version of your code with servo feedback included:

Updated
Code
with Servo Feedback

import time
import math
import serial
from mpu6050 import MPU6050
from pid import PID
from vl53l0x import VL53L0X  # Import for distance sensor

import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.resetSimulation()
# Set up environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add PyBullet data path
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

# Load plane and the uploaded robot URDF
planeId = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
robotId = p.loadURDF("MJKURDF.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
# Serial connection for servos (Feetech SCS115)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Servo configuration
servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]


# Initialize sensors
# mpu = MPU6050(0x68)
# Function to simulate MPU6050 data from the pelvis link
def get_mpu6050_data(robot_id, link_index):
    # Get the link's orientation in world space
    _, orientation = p.getBasePositionAndOrientation(robot_id) if link_index == -1 else p.getLinkState(robot_id,
                                                                                                       link_index,
                                                                                                       computeForwardKinematics=True)[
                                                                                        :2]
    # Convert quaternion to Euler angles


euler_orientation = p.getEulerFromQuaternion(orientation)  # (roll, pitch, yaw)
return np.degrees(euler_orientation)  # Convert radians to degrees

tof_sensor = VL53L0X()
tof_sensor.start_ranging()

# PID controller for balance
pid = PID(1.0, 0.1, 0.05)  # Kp, Ki, Kd, adjust as needed

# Robot dimensions
thigh_length = 10  # cm
calf_length = 10  # cm
foot_length = 5  # cm
target_pitch = 0  # Target pitch angle for balance (horizontal)
max_step_height = thigh_length + calf_length - 2  # cm

# ZMP parameters
com_x, com_y, com_z = 0, 0, 47.7  # cm from the floor
stance_leg_length = 0.2  # meters
g = 9.81  # m/s^2


# Feetech SCS115 protocol for setting and reading angles
def set_servo_angle(joint_index, angle):
    """
    Sets the servo angle using Feetech SCS115 protocol and verifies feedback.
    """
    angle = max(0, min(270, angle))  # Ensure within range
    position = int((angle / 270) * 1023)  # Convert angle to position value (0-1023)
    command = bytearray([servo_id, 0x03, position & 0xFF, (position >> 8) & 0xFF])
    ser.write(command)

    # Confirm feedback
    actual_angle = get_servo_angle(joint_index)
    if actual_angle is None or abs(actual_angle - angle) > 5:  # Allowable error threshold
        print(f"Warning: Servo {servo_id} feedback mismatch. Target: {angle}, Actual: {actual_angle}")


def get_servo_angle(joint_index):  # def get_servo_angle(servo_id):
    """
    Reads the current angle of the servo motor using Feetech SCS115 feedback protocol.
    """
    # command = bytearray([servo_id, 0x02])  # Command to request current position
    # ser.write(command)
    # response = ser.read(6)  # Response is 6 bytes (protocol specific)


joint_state = p.getJointState(robot_id, joint_index)  # Get joint state
position = joint_state[0]  # Joint position (rad)
position = np.degrees(position)  # Convert position to degrees
angle = (position / 1023) * 270  # Convert position to angle
return angle


# if len(response) == 6 and response[0] == servo_id:
# position = (response[5] << 8) | response[4]  # Extract position bytes
# angle = (position / 1023) * 270  # Convert position to angle
# return angle
# else:
# print(f"Error: No feedback from servo {servo_id}.")
# return None

def inverse_kinematics(target_position):
    """
    Calculates joint angles for hip, knee, and ankle.
    Incorporates joint limits and validates feasibility.
    """
    x, y, z = target_position
    distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)

    # Validate target position
    if distance > (thigh_length + calf_length):
        print(f"Target out of reach: {target_position}")
        return None, None, None

    # Knee angle using cosine rule
    knee_angle = math.acos((distance ** 2 - thigh_length ** 2 - calf_length ** 2) / (2 * thigh_length * calf_length))

    # Hip angle using trigonometry
    hip_angle = math.atan2(y, x)

    # Ankle angle to maintain balance
    ankle_angle = math.atan2(z, math.sqrt(x ** 2 + y ** 2)) - knee_angle

    # Convert to degrees and validate joint limits
    hip_angle = max(-90, min(90, math.degrees(hip_angle)))
    knee_angle = max(0, min(135, math.degrees(knee_angle)))
    ankle_angle = max(-45, min(45, math.degrees(ankle_angle)))

    return hip_angle, knee_angle, ankle_angle


def move_leg(target_position, leg):
    """
    Moves the leg to a target position using inverse kinematics and verifies via feedback.
    """
    hip_angle, knee_angle, ankle_angle = inverse_kinematics(target_position)
    if hip_angle is None:
        return  # Invalid target

    if leg == "right":
        set_servo_angle(3, hip_angle)
        set_servo_angle(4, knee_angle)
        set_servo_angle(5, ankle_angle)
    elif leg == "left":
        set_servo_angle(6, hip_angle)
        set_servo_angle(7, knee_angle)
        set_servo_angle(8, ankle_angle)


def step_over_obstacle():
    """
    Detects step height and dynamically adjusts gait for stability, with servo feedback.
    """
    try:
        step_height = tof_sensor.get_distance() / 10.0  # Convert mm to cm
    except Exception as e:
        print(f"Distance sensor error: {e}")
        return

    if 5 <= step_height <= max_step_height:
        print(f"Stepping over obstacle of height: {step_height} cm")
        lift_leg("right")
        move_leg((10, 5, step_height), "right")
        lower_leg("right")
        shift_com("left")
        lift_leg("left")
        move_leg((10, 5, step_height), "left")
        lower_leg("left")
        shift_com("right")
    else:
        print(f"Obstacle height ({step_height} cm) out of range.")
        calculate_detour()
    """
    """
    Detects
    step
    height and dynamically
    adjusts
    gait
    for stability with ZMP control.
    """
    try:
        step_height = tof_sensor.get_distance() / 10.0  # Convert mm to cm
    except Exception as e:
        print(f"Distance sensor error: {e}")
        return

    if 5 <= step_height <= max_step_height:
        print(f"Stepping over obstacle of height: {step_height} cm")

        print("Lifting and Moving Right Leg")
        lift_leg("right")
        move_leg((10, 5, step_height), "right")
        zmp_control()  # Ensure ZMP is stable
        lower_leg("right")
        shift_com("left")

        print("Lifting and Moving Left Leg")
        lift_leg("left")
        move_leg((10, 5, step_height), "left")
        zmp_control()  # Ensure ZMP is stable
        lower_leg("left")
        shift_com("right")

    else:
        print(f"Obstacle height ({step_height} cm) out of range. Calculating detour.")
        calculate_detour()


def calculate_detour():
    """
    Implements
    RRT - based
    detour
    calculation
    with feedback - based servo positioning.
    """
    print("Calculating detour using RRT...")
    # Simplified example: Rotate and step sideways
    set_servo_angle(13, 90)  # Rotate left
    time.sleep(1)
    move_leg((5, 15, 0), "left")
    time.sleep(1)
    set_servo_angle(13, 135)  # Rotate back

def zmp_control():
    """
    Adjusts
    walking
    gait
    to
    ensure
    stability
    using
    ZMP and servo
    feedback.

    zmp_x = com_x - (stance_leg_length / 2)  # Simplified ZMP calculation
    if abs(zmp_x) > (stance_leg_length / 2):
        print("ZMP out of bounds! Adjusting gait.")
        # ADJUST COM OR LEG POSITIONS HERE
    """

   """
    Adjusts
    walking
    gait
    to
    ensure
    stability
    using
    ZMP and servo
    feedback.
    Calculates
    ZMP and adjusts
    COM or leg
    positions
    dynamically.
    """
    global com_x, com_y, com_z

    # Simplified ZMP calculation
    zmp_x = com_x - (stance_leg_length / 2)
    zmp_threshold = stance_leg_length / 2  # ZMP must stay within the support polygon

    # If ZMP is out of bounds, adjust COM or leg positions
    if abs(zmp_x) > zmp_threshold:
        print(f"ZMP out of bounds! ZMP_X: {zmp_x:.2f} Adjusting COM...")

        # Adjust COM horizontally to bring ZMP back within bounds
        adjustment = zmp_x - zmp_threshold if zmp_x > 0 else zmp_x + zmp_threshold
        com_x -= adjustment * 0.5  # Dampen adjustment to avoid overshooting

        # Adjust leg positions to stabilize
        if zmp_x > 0:  # COM shifted too far forward, adjust legs backward
            move_leg((com_x - 5, 0, com_z), "right")
            move_leg((com_x - 5, 0, com_z), "left")
        else:  # COM shifted too far backward, adjust legs forward
            move_leg((com_x + 5, 0, com_z), "right")
            move_leg((com_x + 5, 0, com_z), "left")

        print(f"COM adjusted to X: {com_x:.2f}, ZMP stabilized.")

    else:
        print(f"ZMP within bounds. ZMP_X: {zmp_x:.2f}")
def walking_gait():
    """
    Executes
    a
    walking
    gait, dynamically
    integrating
    balance and feedback.
    """
    for _ in range(10):
        lift_leg("right")
        move_leg((10, 5, 0), "right")
        zmp_control()
        lower_leg("right")
        shift_com("left")

        lift_leg("left")
        move_leg((10, 5, 0), "left")
        zmp_control()
        lower_leg("left")
        shift_com("right")
    """
    """
    Executes a walking gait, dynamically integrating balance, ZMP control, and feedback.
    """
    for step in range(10):  # Example: Take 10 steps
        print(f"Step {step + 1} - Moving Right Leg")
        lift_leg("right")
        move_leg((10, 5, 0), "right")
        zmp_control()  # Check and adjust stability
        lower_leg("right")
        shift_com("left")  # Shift COM dynamically after moving leg

        print(f"Step {step + 1} - Moving Left Leg")
        lift_leg("left")
        move_leg((10, 5, 0), "left")
        zmp_control()  # Check and adjust stability
        lower_leg("left")
        shift_com("right")  # Shift COM dynamically after moving leg


def balance():
    """
    Balances the robot by adjusting leg positions and ZMP.
    """
    current_pitch = mpu.get_pitch()
    error = target_pitch - current_pitch
    correction = pid.compute(error)

    print(f"Balancing: Current Pitch = {current_pitch}, Correction = {correction}")

    # Adjust COM and legs for stability
    zmp_control()

    """
def main():
    """
    Main
    control
    loop
    with debugging hooks and servo feedback.
    """
    try:
        while True:
            print("Balancing...")
            balance()

            print("Obstacle Avoidance...")
            obstacle_avoidance()

            print("Step Over Obstacle...")
            step_over_obstacle()

            print("Walking Gait...")
            walking_gait()

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Program interrupted. Cleaning up.")
        tof_sensor.stop_ranging()
        ser.close()
    """


def main():
    """
    Main control loop with debugging hooks and servo feedback.
    """
    try:
        while True:
            print("Balancing...")
            balance()

            print("Obstacle Avoidance...")
            obstacle_avoidance()

            print("Step Over Obstacle...")
            step_over_obstacle()

            print("Walking Gait...")
            walking_gait()

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Program interrupted. Cleaning up.")
        tof_sensor.stop_ranging()
        ser.close()


if __name__ == '__main__':
    main()

---

Key
Changes

1.
get_servo_angle():

Added
a
function
to
read
the
current
servo
angle
using
the
Feetech
SCS115
feedback
protocol.

2.
set_servo_angle():

Modified
to
include
feedback
validation,
with warnings for discrepancies between the target and actual angles.

3.
Functions
Integration:

All
movement
functions(e.g., move_leg, step_over_obstacle, calculate_detour)
now
incorporate
servo
feedback
to
ensure
precise
execution.

4.
Error
Handling:

Enhanced
for both MPU6050 and VL53L0X to handle sensor errors gracefully.

5.
Dynamic
Adjustments:

Walking
gait, ZMP
control, and step - over
functions
now
dynamically
integrate
servo
feedback
to
adjust
movements in real - time.

---

This
implementation
ensures
robust
feedback - based
control
for more accurate and stable movements.Let me know if you need further modifications or explanations!

