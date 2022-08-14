import mechanics as mech
import camera
import atexit

import numpy as np
import math
# Init camera
cam = camera.Camera()

### GEOMETRY OF ARM ###
SERVO_ARM_LENGTH = 45
SERVO_ARM_ENDPOINT = (40.11, -51.64)
BALL_ARM_LENGTH = 54
BALL_ARM_OFFSET_CENTER = 85.18
MIN_SERVO_ANGEL_DEG = -60  # Servo angel (arm pointing upwards) from parallel with ground
MIN_PLANE_ANGEL_DEG = -26.23  # Calcualted from model. -24.2 according to F360, but ... 
MAX_SERVO_ANGEL_DEG = 47  # Servo angel (arm pointing downwards) from parallel with ground
MAX_PLANE_ANGEL_DEG = 24.63  # Calcualted from model. 25.2 deg according to F360, but ...

### GEOMETRY OF disk ###
RADIUS = 125


### INITIALIZE SERVO MOTORS ###
# All angles are measured as zero
# when the servo arm is horizontal
x_servo = mech.MyServo(21, -87-4, 52-4, 860, 2080)
atexit.register(x_servo.detach)
y_servo = mech.MyServo(20, -87-4, 56-4, 2080, 850)
atexit.register(y_servo.detach)

# Define the mechanics controlling the motion of the disk in the XZ-plane
x_arm = mech.ArmMechanics("x", SERVO_ARM_LENGTH, SERVO_ARM_ENDPOINT, BALL_ARM_LENGTH, BALL_ARM_OFFSET_CENTER,
MIN_SERVO_ANGEL_DEG, MIN_PLANE_ANGEL_DEG, MAX_SERVO_ANGEL_DEG, MAX_PLANE_ANGEL_DEG, servo=x_servo)
# Set the current value of the servo to match a flat plane
x_arm.zero = x_arm.calc_servo_angel(0)

# Define the mechanics controlling the motion of the disk in the YZ-plane
y_arm = mech.ArmMechanics("y", SERVO_ARM_LENGTH, SERVO_ARM_ENDPOINT, BALL_ARM_LENGTH, BALL_ARM_OFFSET_CENTER,
MIN_SERVO_ANGEL_DEG, MIN_PLANE_ANGEL_DEG, MAX_SERVO_ANGEL_DEG, MAX_PLANE_ANGEL_DEG, servo=y_servo)
# Set the current value of the servo to match a flat plane
y_arm.zero =  y_arm.calc_servo_angel(0)

# Define the plane
plane = mech.Plane(RADIUS, x_arm, y_arm)
plane.update()

# Consider saving what the camera detects as the center of the plate
# at this point, since it will not change. Then, later,

if __name__ == '__main__':
    while True:
        # Get the frame from the camera
        cam.next_frame()
        #cam.simulate()
        # Save the current positions of the ball
        # and plate to the data-variable
        pos_data = cam.get_positions()
        # Correct the position of the ball
        plane.correct_ball(pos_data)


