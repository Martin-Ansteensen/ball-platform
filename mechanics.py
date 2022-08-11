from cmath import sqrt
import math as m
import numpy as np
import pigpio
from time import sleep
import atexit

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

# Servo handler
pwm = pigpio.pi()

class ArmMechanics:
    """ Class responsible for calculating the angle of a line. An arm
    controls one axis of rotation of the plane, so therefore this calculation
    is two-dimensional. The angle of a line is measured against a horisonatal line.
    The same goes for the servo angel """

    def __init__(self, axis, L1, J1_ROT_P, L2, R, min_servo_angel_deg, min_line_angel_deg, max_servo_angel_deg, max_line_angel_deg, servo):
        
        # Axis
        self.axis = axis

        # Servo object
        self.servo = servo

        # Angel of the servo motor
        self.current_servo_angel_rad = 0

        # Length of arm attached to servo. Connects J1 and J2
        self.L1 = L1
        # Coordinate for joint of rotation (servo center) of J1
        self.J1_ROT_P = np.array(J1_ROT_P)
        # Length of arm attached to the end of the servo arm
        # and the bottom of the platform
        # Connect J2 and J3
        self.L2 = L2
        # Distance from center of platform to J3 (connection with L2)
        self.R = R
        
        # Joint limits
        self.min_servo_angel_rad = self.to_rad(min_servo_angel_deg)
        self.min_line_angel_rad = self.to_rad(min_line_angel_deg)
        self.max_servo_angel_rad = self.to_rad(max_servo_angel_deg)
        self.max_line_angel_rad = self.to_rad(max_line_angel_deg)

    def calc_line_angel(self, servo_angel):
        """ Returns the angle of the line 
        in radians calculated from the angel of the servo. 
        Takes servo angel in rad as input """

        # Throw exception if the desired servo-angel lies
        # out of range

        # The displacement of J2 due to rotation of the servo
        J2_displacement = np.array([m.cos(servo_angel)*self.L1, m.sin(servo_angel)*self.L1])
        # End coordinate for end of servo arm (J2)
        J2_ROT_P = self.J1_ROT_P + J2_displacement
        # Calculate the slope of the line
        angel_line_constant_part = -0.5*m.pi*np.sign(J2_ROT_P[1])*(np.sign(self.R) + np.sign(J2_ROT_P[0])) + m.atan(J2_ROT_P[1]/J2_ROT_P[0])
        sqrt_part = sqrt(J2_ROT_P[0]**2 + J2_ROT_P[1]**2)*(self.L2**2 - self.R**2 - J2_ROT_P[0]**2 - J2_ROT_P[1]**2)
        angel_line_variable_part = m.acos(sqrt_part.real/(abs(self.R)*2*(J2_ROT_P[0]**2 + J2_ROT_P[1]**2))) 
        
        # Calculate the two possible angels

        # This one is the second solution, does not work for our application
        # v1 = angel_line_constant_part + angel_line_variable_part 

        # Calculates the angle of the plane in radians.
        angel_line = angel_line_constant_part - angel_line_variable_part
        # Correction that seems to work
        angel_line_corrected = angel_line + self.to_rad(0.6)
        # print(angel_line_corrected*180/m.pi)
        return angel_line_corrected

    def calc_servo_angel(self, line_angel_rad, min_servo_rad=None, max_servo_rad=None, num_decimals = 2, last_servo_angel_rad=None):
        """ Returns a corresponding servo angel calculated 
        from an angel of the line. Returns servo angel in 
        radians. Takes angel of line in degrees. This function is
        recursive and is basically doing a binary search """

        # Throw exception if the desired line-angel lies
        # out of range

        # Define the search range for this run
        if min_servo_rad == None or max_servo_rad == None:
            # If it is the first run; use the whole range of
            # the servo motor
            min = self.min_servo_angel_rad
            max = self.max_servo_angel_rad
        else:
            # Use values given from last run
            min = min_servo_rad
            max = max_servo_rad

        # We guess that the servo angel we are looking for lies
        # in the middle of the range we have defined
        guess_servo_angel_rad = (min + max)*0.5
        # Calculate what line-angel this servo-angel gives
        return_line_angel_rad = self.calc_line_angel(guess_servo_angel_rad)
        # Check if have got the close enough to the line-angel we are looking for, 
        # or if the search has stalled and will not reach the amount of precision we want
        if round(return_line_angel_rad, num_decimals) ==  round(line_angel_rad, num_decimals) or last_servo_angel_rad == guess_servo_angel_rad:
            # The search has stalled and will not reach the amount of precision we want
            if last_servo_angel_rad == guess_servo_angel_rad:
                print("Stopped search because equal value was obtained in two consecutive runs")
            # Return the servo-angel that corresponds to the line-angel (initial input)
            print("A servo angel ", self.to_deg(guess_servo_angel_rad), "for " + self.axis + 
            "-axis gives the following angle of the plane", self.to_deg(return_line_angel_rad), "in the "+ self.axis + "z-plane")
            return guess_servo_angel_rad

        # If the calculated line-angel is smaller than the desired line-angel 
        # we need a larger servo-angel, and therefore define a new range excluding
        # the range we know will not work
        elif return_line_angel_rad < line_angel_rad:
            min_param = guess_servo_angel_rad
            max_param = max
        
        # The calculated line-angel is greater than the desired line-angel 
        # we need a smaller servo-angel, and therefore define a new range excluding
        # the range we know will not work
        else:
            min_param = min
            max_param = guess_servo_angel_rad

        # Run a new search (recursive)
        return self.calc_servo_angel(line_angel_rad, min_param, max_param, num_decimals, guess_servo_angel_rad)

    def calc_axis_plane_point(self, angel_line):
        """ Calculates the point of contact between the plane and L2 in 2D.
        To convert the point to 3D one add zero to the x or y coordinate depending 
        on which plane the arm operates inInput angel in radians """
        return np.array([self.R*m.cos(angel_line), self.R*m.sin(angel_line)])

    def to_rad(self, deg):
        """ Converts angel from degrees to radians"""
        rad = deg*m.pi/180
        return rad

    def to_deg(self, rad):
        """ Converts angel from radians to degrees """
        deg = rad*180/m.pi
        return deg   

    def set_servo_angel(self, angel_rad):
        """ Moves the servo to the desired position and updates
        its position variable """

        # Do motor shit
        self.servo.moveToDeg(self.to_deg(angel_rad), wait=True)
        #
        print("Angle for " + self.axis + "-servo is ", self.to_deg(angel_rad))
        self.current_servo_angel_rad = angel_rad
    

class Plane:
    def __init__(self, radius, x_arm, y_arm):
        self.center_pnt = [0,0,0]
        # Point of contact between the disk and the x-axis arm
        self.x_point = [None, None, None]
        # Point of contact between the disk and the y-axis arm
        self.y_point = [None, None, None]
        self.normal_vector = [None, None, None]
        self.radius = radius

        self.x_arm = x_arm
        self.y_arm = y_arm

        self.x_angel_rad = None
        self.y_angel_rad = None

    def update(self):
        """ Updates its x- and y points. Updates normal vector """
        self.x_angel_rad = self.x_arm.calc_line_angel(self.x_arm.current_servo_angel_rad)
        self.y_angel_rad = self.y_arm.calc_line_angel(self.y_arm.current_servo_angel_rad)

        # Get the point in 2D
        self.x_point = self.x_arm.calc_axis_plane_point(self.x_angel_rad)
        # Convert to 3D. Since this is the x-arm it operates in the XZ-plane, 
        # hence the y-coordinate is always zero (this is an assumptoin)
        self.x_point = [self.x_point[0], 0, self.x_point[1]]
        self.y_point = self.y_arm.calc_axis_plane_point(self.y_angel_rad)
        self.y_point = [0, self.y_point[0], self.y_point[1]]

        self.normal_vector = self.calc_normal_vector(self.center_pnt, self.x_point, self.y_point)

    def calc_normal_vector(self, p1, p2, p3):

        p1, p2, p3 = np.array(p1), np.array(p2), np.array(p3)
        u = p1-p2
        v = p1-p3
        n_vec = np.cross(u,v)
        print("Normal vector of plane", n_vec)
        return n_vec
    
    def set_plane_angels(self, x_angel_rad, y_angel_rad):
        """ Moves the plane to a certain angel in the XZ-plane
        and the YZ-plane """
        x_servo_rad = self.x_arm.calc_servo_angel(x_angel_rad)
        y_servo_rad = self.y_arm.calc_servo_angel(y_angel_rad)
        
        self.x_arm.set_servo_angel(x_servo_rad)
        self.y_arm.set_servo_angel(y_servo_rad)

        self.update()
    
    def test(self):
        """ Runs a series of commands to ensure that 
        the plane is working as intended"""
        print("Slow test")

        print("Moving to horizontal position")
        plane.set_plane_angels(0, 0)
        sleep(1)
        print("Giving disk -20 deg rotation in x, (plane pointing away from x-servo")
        plane.set_plane_angels(-20*m.pi/180, 0)
        sleep(1)
        print("Giving disk -20 deg rotation in y, (plane pointing away from x-servo")
        plane.set_plane_angels(0,-20*m.pi/180)
        sleep(1)
        print("Moving to horizontal position")
        plane.set_plane_angels(0, 0)
    #### NEW FUNCTION ####
    # Camera has 2d vector for direction of slope. The amount of slope should be the error*k, and this would be the z-component of the vector. This function should position the plane along that vector

class Ball:
    def __init__(self):
        self.mass = None
        self.radius = None
        self.position = [None, None, None]
    
    def calc_position(self, time_intervall):
        """ Calculates the position of the ball travelling in a
        straight line from its current position after a given
        time_intervall (given in seconds)"""
        pass
    
    def acceleration(self, normal_vector):
        """ Calculates the acceleration of the ball given the 
        normal vector of the plane the ball is on"""
        pass

class MyServo():
    def __init__(self, pin, min_deg, max_deg, min_pulse, max_pulse):
        self.pin = pin
        self.min_pulse = min_pulse
        self.min_deg = min_deg
        self.max_pulse = max_pulse
        self.max_deg = max_deg 
        self.last_pulse = 0
        # Angle that makes the arm horizontal
        self.zero_deg = 0

        # Move to horizontal position of the plane
        print("Move to horizontal position of the plane")
        self.moveToDeg(0)
        sleep(1)

    def moveToDeg(self, deg, wait = False):
        # Convert deg to pulse
        pulse = self.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)
        # Change the zero position from horizontal of plane to
        # horizontal of servo arm
        #pulse_correction = self.map(self.zero_deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)
        pulse_corrected = self.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)

        # Calculate sleep time
        sleep_time = abs(self.last_pulse-pulse_corrected)*0.8/1230 #0.8
        pwm.set_servo_pulsewidth(self.pin, pulse_corrected)
        self.last_pulse = pulse_corrected
        print("Moving servo to", deg , "measured from when the servo arm is horizontal")
        if wait:
            sleep(sleep_time)
        
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def detach(self):
        pwm.set_servo_pulsewidth(self.pin, 0)
    

### INITIALIZE SERVO MOTORS ###
# All angles are measured with zero as
# when the servo arm is horizontal

x_servo = MyServo(21, -87-4, 52-4, 860, 2080)
atexit.register(x_servo.detach)
y_servo = MyServo(20, -87-4, 56-4, 2080, 850)
atexit.register(y_servo.detach)

# Define the mechanics controlling the motion of the disk in the XZ-plane
x_arm = ArmMechanics("x", SERVO_ARM_LENGTH, SERVO_ARM_ENDPOINT, BALL_ARM_LENGTH, BALL_ARM_OFFSET_CENTER,
MIN_SERVO_ANGEL_DEG, MIN_PLANE_ANGEL_DEG, MAX_SERVO_ANGEL_DEG, MAX_PLANE_ANGEL_DEG, servo=x_servo)
# Set the current value of the servo to match a flat plane
x_arm.zero = x_arm.calc_servo_angel(0)

# Define the mechanics controlling the motion of the disk in the YZ-plane
y_arm = ArmMechanics("y", SERVO_ARM_LENGTH, SERVO_ARM_ENDPOINT, BALL_ARM_LENGTH, BALL_ARM_OFFSET_CENTER,
MIN_SERVO_ANGEL_DEG, MIN_PLANE_ANGEL_DEG, MAX_SERVO_ANGEL_DEG, MAX_PLANE_ANGEL_DEG, servo=y_servo)
# Set the current value of the servo to match a flat plane
y_arm.zero =  y_arm.calc_servo_angel(0)

# Define the plane
plane = Plane(RADIUS, x_arm, y_arm)
plane.update()
if __name__=='__main__':
    plane.test()

