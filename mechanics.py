from cmath import sqrt
import math as m
import numpy as np
import pigpio
from time import sleep
import atexit
import logging
import json

with open("config.json", "r") as f:
    config_data = json.load(f)["mechanics"]
    f.close()

### GEOMETRY OF ARM ###
# Length of servo arm (from center of rotation in joints)
SERVO_ARM_LENGTH = config_data["geometry"]["servo_arm_length"]
# Point where servo arm connects to servo
SERVO_ARM_ENDPOINT = config_data["geometry"]["servo_arm_endpoint"]
# Length of arm going from servo arm to disk
BALL_ARM_LENGTH = config_data["geometry"]["ball_arm_length"]
# Distance from arm to center of disk (stirct length, not pytagoras)
BALL_ARM_OFFSET_CENTER = config_data["geometry"]["ball_arm_offset_center"]
# Servo angel (arm pointing upwards) from parallel with ground
MIN_SERVO_ANGEL_DEG = config_data["geometry"]["min_servo_angel_deg"] 
# Calcualted from model. -24.2 according to F360, but ... 
MIN_PLANE_ANGEL_DEG = config_data["geometry"]["min_plane_angel_deg"]  
# Servo angel (arm pointing downwards) from parallel with ground
MAX_SERVO_ANGEL_DEG = config_data["geometry"]["max_servo_angel_deg"]  
# Calcualted from model. 25.2 deg according to F360, but ...
MAX_PLANE_ANGEL_DEG = config_data["geometry"]["max_plane_angel_deg"]  

### GEOMETRY OF DISK ###
RADIUS = config_data["geometry"]["disk_radis"]

### SERVO CONSTANTS ###
# All angels are measured from when
# the servo arm is horizontal
X_SERVO_PIN = config_data["x_servo"]["servo_pin"]
X_SERVO_MIN_DEG = config_data["x_servo"]["servo_min_deg"]
X_SERVO_MAX_DEG = config_data["x_servo"]["servo_max_deg"]
X_SERVO_MIN_PULSE = config_data["x_servo"]["servo_min_pulse"]
X_SERVO_MAX_PULSE = config_data["x_servo"]["servo_max_pulse"]

Y_SERVO_PIN = config_data["y_servo"]["servo_pin"]
Y_SERVO_MIN_DEG = config_data["y_servo"]["servo_min_deg"]
Y_SERVO_MAX_DEG = config_data["y_servo"]["servo_max_deg"]
Y_SERVO_MIN_PULSE = config_data["y_servo"]["servo_min_pulse"]
Y_SERVO_MAX_PULSE = config_data["y_servo"]["servo_max_pulse"]


class MyOperations():
    """ Class containing useful math operations"""
    def __init__(self):
        pass

    def to_rad(self, deg):
        """ Converts angel from degrees to radians"""
        rad = deg*m.pi/180
        return rad

    def to_deg(self, rad):
        """ Converts angel from radians to degrees """
        deg = rad*180/m.pi
        return deg   

    def norm_vec_plane(self, p1, p2, p3):
        """ Finds normal vector of three points (a plane)"""
        p1, p2, p3 = np.array(p1), np.array(p2), np.array(p3)
        u = p1-p2
        v = p1-p3
        n_vec = np.cross(u,v)
        logging.debug("Normal vector of plane", n_vec)
        return n_vec
    
    def angle_between_vecs(self, u, v):
        """ Returns angle bewteen vectors in radians """
        u = np.array(u)
        v = np.array(v)
        
        # Find length of vectors
        u_len = np.linalg.norm(u)
        v_len = np.linalg.norm(v)
        # Calculate angle between vectors
        angle = np.arccos(np.dot(u,v)/(u_len*v_len))
        # The angles between two vectors can never 
        # be greater than 180 deg (pi radians)
        if angle >= m.pi:
            angle = 2*m.pi - angle
        if round(angle, 2) == 3.14:
            angle = 0
        # Return the angle in radians
        return angle

    def map(self, x, in_min, in_max, out_min, out_max):
        """ Map input value on one scale to another. """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Create math instance
calc = MyOperations()

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
        self.min_servo_angel_rad = calc.to_rad(min_servo_angel_deg)
        self.min_line_angel_rad = calc.to_rad(min_line_angel_deg)
        self.max_servo_angel_rad = calc.to_rad(max_servo_angel_deg)
        self.max_line_angel_rad = calc.to_rad(max_line_angel_deg)

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
        angel_line_corrected = angel_line + calc.to_rad(0.6)
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
                logging.debug("Stopped search because equal value was obtained in two consecutive runs")
            # Return the servo-angel that corresponds to the line-angel (initial input)
            logging.debug("A servo angel ", calc.to_deg(guess_servo_angel_rad), "for " + self.axis + 
            "-axis gives the following angle of the plane", calc.to_deg(return_line_angel_rad), "in the "+ self.axis + "z-plane")
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
        on which plane the arm operates in. Input angel in radians """
        return np.array([self.R*m.cos(angel_line), self.R*m.sin(angel_line)])

    def set_servo_angel(self, angel_rad, wait_for_finish = False):
        """ Moves the servo to the desired position and updates
        its position variable """

        # Move servo
        self.servo.moveToDeg(calc.to_deg(angel_rad), wait=wait_for_finish)
        logging.debug("Angle for " + self.axis + "-servo is ", calc.to_deg(angel_rad))
        # Save current position
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

        self.ball_detected_last_run = False

        profile_names = ["linear", "cubic", "mix"]


        pid_profiles = {
            profile_names[0]: {
                "kp": 2,
                "ki": 0.05,
                "kd": 35,
                "factor": 0.0001
            },
            profile_names[1]: {
                "kp": 0.2,
                "ki": 0.02,
                "kd": 2,
                "factor": 0.00001
            },
            profile_names[2]: {
                "kp": 25,
                "ki": 500,
                "kd": 2,
                "factor": 0.00001
            }
        }

        profile = profile_names[2]
        
        Kp = pid_profiles[profile]["kp"]
        Ki = pid_profiles[profile]["ki"]
        Kd = pid_profiles[profile]["kd"]
        factor = pid_profiles[profile]["factor"]

        print(f"Using {profile}-PID Kp: {Kp} Ki: {Ki} Kd: {Kd} and Kr: {factor}")

        if profile == "linear":
            self.x_pid = PID_Controller(Kp, Ki, Kd, factor)
            self.y_pid = PID_Controller(Kp, Ki, Kd, factor)
        
        elif profile == "cubic":
            self.x_pid = Cubic_PID_Controller(Kp, Ki, Kd, factor)
            self.y_pid = Cubic_PID_Controller(Kp, Ki, Kd, factor)

        elif profile == "mix":
            self.x_pid = Mix_PID_Controller(Kp, Ki, Kd, factor)
            self.y_pid = Mix_PID_Controller(Kp, Ki, Kd, factor)



    def update(self):
        """ Updates its x- and y points (calculations) Updates normal vector """
        self.x_angel_rad = self.x_arm.calc_line_angel(self.x_arm.current_servo_angel_rad)
        self.y_angel_rad = self.y_arm.calc_line_angel(self.y_arm.current_servo_angel_rad)

        # Get the point in 2D
        self.x_point = self.x_arm.calc_axis_plane_point(self.x_angel_rad)
        # Convert to 3D. Since this is the x-arm it operates in the XZ-plane, 
        # hence the y-coordinate is always zero (this is an assumptoin)
        self.x_point = [self.x_point[0], 0, self.x_point[1]]
        # Same for y-servo
        self.y_point = self.y_arm.calc_axis_plane_point(self.y_angel_rad)
        self.y_point = [0, self.y_point[0], self.y_point[1]]

        self.normal_vector = calc.norm_vec_plane(self.center_pnt, self.x_point, self.y_point)
    
    def set_plane_from_angels(self, x_angel_rad, y_angel_rad, servo_wait_for_finish = False):
        """ Moves the plane to a certain angel in the XZ-plane
        and the YZ-plane (moves servo) """
        x_servo_rad = self.x_arm.calc_servo_angel(x_angel_rad)
        y_servo_rad = self.y_arm.calc_servo_angel(y_angel_rad)
        
        self.x_arm.set_servo_angel(x_servo_rad, wait_for_finish=servo_wait_for_finish)
        self.y_arm.set_servo_angel(y_servo_rad, wait_for_finish=servo_wait_for_finish)

        self.update()
    
    def set_plane_from_vecs(self, u, v):
        " Moves plane so that its normal vector is the cross product of the two given vector"
        # Calc norm vec
        norm_vec = np.cross(u, v)
        # Get x and y angel of plane
        x_rad = np.arcsin(norm_vec[0]/norm_vec[2])
        y_rad = np.arcsin(norm_vec[1]/norm_vec[2])
        # Set plane according to these angles
        self.set_plane_from_angels(x_rad, y_rad)

    def correct_ball(self, pos_data, target_pos):
        # Check if ball is detected
        if pos_data["ball"]["x"] == None:
            # The ball is not detected.
            # Move the plane to zero incline
            # and reset the pid controllers
            self.set_plane_from_angels(0, 0)
            self.x_pid.reset()
            self.y_pid.reset()
            self.ball_detected_last_run = False
            return

        # Calculate error for each axis
        x_error = (pos_data["ball"]["x"] - target_pos["x"])*-1
        y_error = (pos_data["ball"]["y"] - target_pos["y"])*-1
        
        # Compute a angle of the plane that will get the ball
        # closer to its target
        x_adjust = self.x_pid.compute(x_error)
        y_adjust = self.y_pid.compute(y_error)

        # Only adjust the plane if the ball was detected
        # last frame as well. The computations made on
        # the first frame of the ball are not correct
        if not self.ball_detected_last_run:
            self.ball_detected_last_run = True
        else:
            self.set_plane_from_angels(x_adjust, y_adjust)
            
    def test(self):
        """ Runs a series of commands to ensure that 
        the plane is working as intended"""
        logging.info("Slow test")

        logging.info("Moving to horizontal position")
        self.set_plane_from_angels(0, 0, servo_wait_for_finish = True)
        sleep(1)
        logging.info("Giving disk -20 deg rotation in x, (plane pointing away from x-servo")
        self.set_plane_from_angels(-20*m.pi/180, 0, servo_wait_for_finish = True)
        sleep(1)
        logging.info("Giving disk -20 deg rotation in y, (plane pointing away from x-servo")
        self.set_plane_from_angels(0,-20*m.pi/180, servo_wait_for_finish = True)
        sleep(1)
        logging.info("Moving to horizontal position")
        self.set_plane_from_angels(0, 0, servo_wait_for_finish = True)

class MyServo():
    # Servo handler
    pwm = pigpio.pi()
    
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
        self.moveToDeg(0, wait=True)
        print("Servo move init (horizontal servo arm), zero_deg is not set yet: ", self.zero_deg)


    def moveToDeg(self, deg, wait = False):
        deg += self.zero_deg
        # Don't let the servo move out of its boundries
        if deg > self.max_deg:
            deg = self.max_deg
        if deg < self.min_deg:
            deg = self.min_deg
            
        pulse_corrected = calc.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)

        # Calculate sleep time
        sleep_time = abs(self.last_pulse-pulse_corrected)*0.8/1230 #0.8
        self.pwm.set_servo_pulsewidth(self.pin, pulse_corrected)
        self.last_pulse = pulse_corrected
        #logging.info(("Moving servo to", deg , "measured from when the servo arm is horizontal"))
        if wait:
            sleep(sleep_time)
        
    def detach(self):
        self.pwm.set_servo_pulsewidth(self.pin, 0)
    
class PID_Controller():
    """ Normal PID-controller using error provided
    for calculations """

    def __init__(self, Kp, Ki, Kd, reduction_factor = 1):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.reduction_factor = reduction_factor

        self.last_error = 0

        self.p = 0
        self.i = 0
        self.d = 0
    
    

    def compute(self, error):
        self.p = error*self.kp
        self.i += error*self.ki
        self.d = (error - self.last_error)*self.kd
        correction = (self.p + self.i + self.d)*self.reduction_factor
        self.last_error = error

        return correction
    
    def reset(self):
        self.p = 0
        self.i = 0
        self.d = 0
        self.last_error = 0

        # When cubing the sign needs to be
        # reimplented

class Cubic_PID_Controller(PID_Controller):
    def __init__(self, Kp, Ki, Kd, reduction_factor = 1):
        super().__init__(Kp, Ki, Kd, reduction_factor)

    def compute(self, error):
        if error == 0:
            error_sign = 0
        else:
            error_sign = error/abs(error)
        
        error *= error*error_sign

        self.p = error*self.kp
        self.i += error*self.ki
        self.d = (error - self.last_error)*self.kd
        correction = (self.p + self.i + self.d)*self.reduction_factor
        self.last_error = error

        return correction

class Mix_PID_Controller(PID_Controller):
    def __init__(self, Kp, Ki, Kd, reduction_factor = 1):
        super().__init__(Kp, Ki, Kd, reduction_factor)
        self.cube_error = 0
        self.last_cube_error = 0

    def compute(self, error):
        if error == 0:
            error_sign = 0
            inverse_error = 0
        else:
            error_sign = error/abs(error)
            # if error < 5 do not do anything
            inverse_error = 1/error
        self.cube_error = error*error*error_sign
        self.sqrt_error = m.sqrt(abs(error))*error_sign
        
        if abs(error) < 10:
            inverse_error = error*(self.kp/self.ki)
            self.i += inverse_error*self.ki
            self.i *= 0.8

        else:
            self.i += inverse_error*self.ki
        
        self.p = error*self.kp
        self.d = (self.cube_error - self.last_cube_error)*self.kd
        correction = (self.p + self.i + self.d)*self.reduction_factor
        self.last_error = error
        self.last_cube_error = self.cube_error

        return correction

    def reset(self):
        self.p = 0
        self.i = 0
        self.d = 0
        self.last_error = 0
        self.last_cube_error = 0

def setup():
    # Initialize servo motors
    x_servo = MyServo(X_SERVO_PIN, X_SERVO_MIN_DEG, X_SERVO_MAX_DEG, X_SERVO_MIN_PULSE, X_SERVO_MAX_PULSE)
    atexit.register(x_servo.detach)
    y_servo = MyServo(Y_SERVO_PIN, Y_SERVO_MIN_DEG, Y_SERVO_MAX_DEG,Y_SERVO_MIN_PULSE, Y_SERVO_MAX_PULSE)
    atexit.register(y_servo.detach)

    # Define the mechanics controlling the motion of the disk in the XZ-plane
    x_arm = ArmMechanics("x", SERVO_ARM_LENGTH, SERVO_ARM_ENDPOINT, BALL_ARM_LENGTH, BALL_ARM_OFFSET_CENTER,
    MIN_SERVO_ANGEL_DEG, MIN_PLANE_ANGEL_DEG, MAX_SERVO_ANGEL_DEG, MAX_PLANE_ANGEL_DEG, servo=x_servo)
    # Find angle of servo that makes plane flat, and 
    # set this as the new zero as servo position
    x_arm.zero_deg = calc.to_deg(x_arm.calc_servo_angel(0)) 

    # Define the mechanics controlling the motion of the disk in the YZ-plane
    y_arm = ArmMechanics("y", SERVO_ARM_LENGTH, SERVO_ARM_ENDPOINT, BALL_ARM_LENGTH, BALL_ARM_OFFSET_CENTER,
    MIN_SERVO_ANGEL_DEG, MIN_PLANE_ANGEL_DEG, MAX_SERVO_ANGEL_DEG, MAX_PLANE_ANGEL_DEG, servo=y_servo)
    # Find angle of servo that makes plane flat, and 
    # set this as the new zero as servo position
    y_arm.zero_deg =  calc.to_deg(y_arm.calc_servo_angel(0))

    # Define the plane
    plane = Plane(RADIUS, x_arm, y_arm)
    plane.update()
    # Make the plane flat
    print("Init plane to zero deg incline")
    plane.set_plane_from_angels(0,0, servo_wait_for_finish = True)
    return plane

if __name__=='__main__':
    # Do setuo
    plane_instance = setup()

    # Test plane by giving it different slopes in many directions
    plane_instance.test()