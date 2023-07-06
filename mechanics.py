from cmath import sqrt
import math as m
import numpy as np
import pigpio
from time import sleep
import atexit
import logging
import json
import os


# Path to current directory
path = os.path.dirname(os.path.abspath(__file__)) + "/"
# Open config file
with open(path + "config.json", "r") as f:
    config_data = json.load(f)["mechanics"]
    f.close()

### GEOMETRY OF ARM ###
# Length of arm connected to servo
L1 = config_data["geometry"]["L1"]
# Center of rotation of servo relative to 
# center of universal joint
J1 = config_data["geometry"]["J1"]
# Length of arm connecting the servo arm 
# to the disk (arm with ball-joints) 
# (distance between J2 and J3)
L2 = config_data["geometry"]["L2"]
# Distance from center of universal joint to the ball joint
# connected to the disk (J3)
D1 = config_data["geometry"]["D1"]
# Servo angle (arm pointing upwards) from parallel with ground
MIN_SERVO_ANGLE_DEG = config_data["geometry"]["min_servo_angle_deg"] 
# Angle of plane when the servo is in the min-angle position
MIN_PLANE_ANGLE_DEG = config_data["geometry"]["min_plane_angle_deg"]  
# Servo angle (arm pointing downwards) from parallel with ground
MAX_SERVO_ANGLE_DEG = config_data["geometry"]["max_servo_angle_deg"]  
# Angle of plane when the servo is in the max-angle position
MAX_PLANE_ANGLE_DEG = config_data["geometry"]["max_plane_angle_deg"]  

### GEOMETRY OF DISK ###
RADIUS = config_data["geometry"]["disk_radius"]

### SERVO CONSTANTS ###
# All angles are measured from when
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

class MathUtils():
    """ Class containing useful math operations"""
    def __init__(self):
        pass

    def norm_vec_plane(self, p1, p2, p3):
        """ Finds normal vector of three points (a plane)"""
        p1, p2, p3 = np.array(p1), np.array(p2), np.array(p3)
        u = p1-p2
        v = p1-p3
        n_vec = np.cross(u,v)
        logging.debug("Normal vector of plane", n_vec)
        return n_vec
    
    def angle_between_vecs(self, u, v):
        """ Returns angle bewteen two vectors (in radians)"""
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
        # 2pi is the same as 0
        if round(angle, 2) == 3.14:
            angle = 0
        return angle

    def map(self, x, in_min, in_max, out_min, out_max):
        """ Maps input value on one scale to another. """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Create math instance
match_utils = MathUtils()

class ArmMechanics:
    """ Class responsible for calculating how the angle of a servo changes
    the angle of the plane/disk (in the same plane as the servo operates),
    and vice versa. A servo controls one axis of rotation of the plane, 
    and the plane/disk is therefore refered to as a line. The angle of a line 
    is measured against a horisonatal line, and the same applies to the servo angle"""

    def __init__(self, axis, L1, J1_ROT_P, L2, D1, min_servo_angle_deg, min_line_angle_deg, max_servo_angle_deg, max_line_angle_deg, servo):
        
        # Axis ("x" or "y")
        self.axis = axis

        # Servo object 
        self.servo: MyServo = servo

        # Angel of the servo motor
        self.current_servo_angle_rad = 0

        self.L1 = L1
        self.J1_ROT_P = np.array(J1_ROT_P)
        self.L2 = L2
        self.D1 = D1
        
        # Joint limits
        self.min_servo_angle_rad = np.deg2rad(min_servo_angle_deg)
        self.min_line_angle_rad = np.deg2rad(min_line_angle_deg)
        self.max_servo_angle_rad = np.deg2rad(max_servo_angle_deg)
        self.max_line_angle_rad = np.deg2rad(max_line_angle_deg)

    def calc_line_angle(self, servo_angle_rad):
        """ Returns the angle of the line given the servo angel.
        Takes servo angle in rad as input.
        Returns angle of line in rad.        
        """

        # The displacement of J2 due to rotation of the servo
        J2_displacement = np.array([m.cos(servo_angle_rad)*self.L1, m.sin(servo_angle_rad)*self.L1])
        # End coordinate for end of servo arm (J2)
        J2_ROT_P = self.J1_ROT_P + J2_displacement
        # Calculate the slope of the line
        
        # angle_line_constant_part = -0.5*m.pi*np.sign(J2_ROT_P[1])*(np.sign(self.D1) + np.sign(J2_ROT_P[0])) + m.atan(J2_ROT_P[1]/J2_ROT_P[0])
        # sqrt_part = (self.D1**2 - self.L2**2 + J2_ROT_P[0]**2 + J2_ROT_P[1]**2)*sqrt(J2_ROT_P[0]**2 + J2_ROT_P[1]**2)/(2*abs(self.D1)*(J2_ROT_P[0]**2 + J2_ROT_P[1]**2))
        # angle_line_variable_part = -m.asin(sqrt_part.real) + 3.58

        angle_line_constant_part = -0.5*m.pi*np.sign(J2_ROT_P[1])*(np.sign(self.D1) + np.sign(J2_ROT_P[0])) + m.atan(J2_ROT_P[1]/J2_ROT_P[0])
        sqrt_part = sqrt(J2_ROT_P[0]**2 + J2_ROT_P[1]**2)*(self.L2**2 - self.D1**2 - J2_ROT_P[0]**2 - J2_ROT_P[1]**2)
        angle_line_variable_part = m.acos(sqrt_part.real/(abs(self.D1)*2*(J2_ROT_P[0]**2 + J2_ROT_P[1]**2))) 

        # Calculate the two possible angles
        # This one is the second solution, does not work for our application
        # v1 = angle_line_constant_part + angle_line_variable_part 

        # Calculates the angle of the plane in radians.
        angle_line = angle_line_constant_part - angle_line_variable_part
        # The constant added was found by testing different values
        angle_line_corrected = angle_line + np.deg2rad(0.6)
        return angle_line_corrected

    def calc_servo_angle(self, line_angle_rad, min_servo_rad=None, max_servo_rad=None, num_decimals = 2, last_servo_angle_rad=None):
        """ Returns servo angel given angle of line (reverse kinematics)
        Takes line of angle in rad.
        This function is recursive and is basically doing a binary search. This works
        because calc_line_angle is a monotonic function
        Retunrns servo angle in rad.
        """

        # Define the search range for this run
        if min_servo_rad == None or max_servo_rad == None:
            # It is the first run; use the whole range of
            # the servo motor
            min = self.min_servo_angle_rad
            max = self.max_servo_angle_rad
        else:
            # Use values given from last run
            min = min_servo_rad
            max = max_servo_rad

        # We guess that the servo angle we are looking for lies
        # in the middle of the range we have defined
        guess_servo_angle_rad = (min + max)*0.5
        # Calculate what line-angle this servo-angle gives
        return_line_angle_rad = self.calc_line_angle(guess_servo_angle_rad)
        # Check if have got the close enough to the line-angle we are looking for, 
        # or if the search has stalled and will not reach the amount of precision we want
        if round(return_line_angle_rad, num_decimals) ==  round(line_angle_rad, num_decimals) or last_servo_angle_rad == guess_servo_angle_rad:
            if last_servo_angle_rad == guess_servo_angle_rad:
                # The search has stalled and will not reach the amount of precision we want
                logging.debug("Stopped search because equal value was obtained in two consecutive runs")
            # Return the servo-angle that corresponds to the line-angle (initial input)
            logging.debug("A servo angle ", np.rad2deg(guess_servo_angle_rad), "for " + self.axis + 
            "-axis gives the following angle of the plane", np.rad2deg(return_line_angle_rad), "in the "+ self.axis + "z-plane")
            return guess_servo_angle_rad

        # If the calculated line-angle is smaller than the desired line-angle 
        # we need a larger servo-angle, and therefore define a new range excluding
        # the range we know will not work
        elif return_line_angle_rad < line_angle_rad:
            min_param = guess_servo_angle_rad
            max_param = max
        
        # The calculated line-angle is greater than the desired line-angle 
        # we need a smaller servo-angle, and therefore define a new range excluding
        # the range we know will not work
        else:
            min_param = min
            max_param = guess_servo_angle_rad

        # Run a new search (recursive)
        return self.calc_servo_angle(line_angle_rad, min_param, max_param, num_decimals, guess_servo_angle_rad)

    def calc_axis_plane_point(self, angle_line):
        """ Calculates the point of contact between the plane and L2 in 2D (J3).
        To convert the point to 3D one add zero to the x or y coordinate depending 
        on which plane the arm operates in. Input angle in radians """
        return np.array([self.D1*m.cos(angle_line), self.D1*m.sin(angle_line)])

    def set_servo_angle(self, angle_rad, wait_for_finish = False):
        """ Moves the servo to the desired position and updates
        its position variable """

        # Move servo
        self.servo.moveToDeg(np.rad2deg(angle_rad), wait=wait_for_finish)
        logging.debug("Angle for " + self.axis + "-servo is ", np.rad2deg(angle_rad))
        # Save current position
        self.current_servo_angle_rad = angle_rad 

class Plane:
    """ Class representing the whole plane, that consists of two
    instances of class ArmMechanics. Is responsible for controlling
    the ball on the plane, given the position of the ball.
    """
    def __init__(self, radius, x_arm, y_arm):
        self.center_pnt = [0,0,0]
        # Point of contact between the disk and the x-axis arm (J3)
        self.x_point = [None, None, None]
        # Point of contact between the disk and the y-axis arm (J3)
        self.y_point = [None, None, None]
        self.normal_vector = [None, None, None]
        self.radius = radius

        self.x_arm: ArmMechanics = x_arm
        self.y_arm: ArmMechanics = y_arm

        self.x_angle_rad = None
        self.y_angle_rad = None

        self.ball_detected_last_run = False
        self.ball_target_point = (None, None)
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
        """ Updates the plane's normal vector (and it's x- and y points """
        self.x_angle_rad = self.x_arm.calc_line_angle(self.x_arm.current_servo_angle_rad)
        self.y_angle_rad = self.y_arm.calc_line_angle(self.y_arm.current_servo_angle_rad)

        # Get the point in 2D
        self.x_point = self.x_arm.calc_axis_plane_point(self.x_angle_rad)
        # Convert to 3D. Since this is the x-arm it operates in the XZ-plane, 
        # hence the y-coordinate is always zero (this is an assumptoin)
        self.x_point = [self.x_point[0], 0, self.x_point[1]]
        # Same for y-servo
        self.y_point = self.y_arm.calc_axis_plane_point(self.y_angle_rad)
        self.y_point = [0, self.y_point[0], self.y_point[1]]

        self.normal_vector = match_utils.norm_vec_plane(self.center_pnt, self.x_point, self.y_point)
    
    def set_plane_from_angles(self, x_angle_rad, y_angle_rad, servo_wait_for_finish = False):
        """ Moves the plane to a certain angle in the XZ-plane
        and the YZ-plane (moves servo) """
        x_servo_rad = self.x_arm.calc_servo_angle(x_angle_rad)
        y_servo_rad = self.y_arm.calc_servo_angle(y_angle_rad)
        
        self.x_arm.set_servo_angle(x_servo_rad, wait_for_finish=servo_wait_for_finish)
        self.y_arm.set_servo_angle(y_servo_rad, wait_for_finish=servo_wait_for_finish)

        self.update()
    
    def set_plane_from_vecs(self, u, v):
        """ Moves plane so that its normal vector is the cross product of the two given vector"""
        # Calc norm vec
        norm_vec = np.cross(u, v)
        # Get x and y angle of plane
        x_rad = np.arcsin(norm_vec[0]/norm_vec[2])
        y_rad = np.arcsin(norm_vec[1]/norm_vec[2])
        # Set plane according to these angles
        self.set_plane_from_angles(x_rad, y_rad)

    def correct_ball(self, pos_data):
        # Get ball target position
        target_pos = self.get_ball_target_point()

        # Check if ball is detected
        if pos_data["ball"]["x"] == None:
            # The ball is not detected.
            # Move the plane to zero incline
            # and reset the pid controllers
            self.set_plane_from_angles(0, 0)
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
            self.set_plane_from_angles(x_adjust, y_adjust)

    def set_ball_target_point(self, point):
        self.ball_target_point = point

    def get_ball_target_point(self):
        return self.ball_target_point


    def test(self):
        """ Runs a series of commands to ensure that 
        the plane is working as intended"""
        logging.info("Slow test")

        logging.info("Moving to horizontal position")
        self.set_plane_from_angles(0, 0, servo_wait_for_finish = True)
        sleep(1)
        logging.info("Giving disk -20 deg rotation in x, (plane pointing away from x-servo")
        self.set_plane_from_angles(-20*m.pi/180, 0, servo_wait_for_finish = True)
        sleep(1)
        logging.info("Giving disk -20 deg rotation in y, (plane pointing away from x-servo")
        self.set_plane_from_angles(0,-20*m.pi/180, servo_wait_for_finish = True)
        sleep(1)
        logging.info("Moving to horizontal position")
        self.set_plane_from_angles(0, 0, servo_wait_for_finish = True)

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
        # Angle that makes the plane horizontal
        # 0 is a dummy value and will be changed after init
        # of the servo object
        self.zero_deg = 0

        # Move to horizontal position of the plane
        self.moveToDeg(0, wait=True)
        print("Servo move init (horizontal servo arm), zero_deg is not set yet: ", self.zero_deg)


    def moveToDeg(self, deg, wait = False):
        """ Moves servo to given deg relative to the servo's
        zero_deg. If wait is True, the function will not return
        until the servo has reached it's endpoint (using an
        approximation, as the servo doesn't have any feedback)"""
        deg += self.zero_deg
        # Don't let the servo move out of its boundries
        if deg > self.max_deg:
            deg = self.max_deg
        if deg < self.min_deg:
            deg = self.min_deg
            
        pulse_corrected = match_utils.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)

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
        logging.info("Lost ball, PID resetting")
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
        logging.info("Lost ball, PID resetting")
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
    x_arm = ArmMechanics("x", L1, J1, L2, D1,
    MIN_SERVO_ANGLE_DEG, MIN_PLANE_ANGLE_DEG, MAX_SERVO_ANGLE_DEG, MAX_PLANE_ANGLE_DEG, servo=x_servo)
    # Find angle of servo that makes plane flat, and 
    # set this as the new zero as servo position
    x_arm.zero_deg = np.rad2deg(x_arm.calc_servo_angle(0)) 

    # Define the mechanics controlling the motion of the disk in the YZ-plane
    y_arm = ArmMechanics("y", L1, J1, L2, D1,
    MIN_SERVO_ANGLE_DEG, MIN_PLANE_ANGLE_DEG, MAX_SERVO_ANGLE_DEG, MAX_PLANE_ANGLE_DEG, servo=y_servo)
    # Find angle of servo that makes plane flat, and 
    # set this as the new zero as servo position
    y_arm.zero_deg =  np.rad2deg(y_arm.calc_servo_angle(0))

    # Define the plane
    plane = Plane(RADIUS, x_arm, y_arm)
    plane.update()
    # Make the plane flat
    print("Init plane to zero deg incline")
    plane.set_plane_from_angles(0,0, servo_wait_for_finish = True)
    return plane

if __name__=='__main__':
    # Do setuo
    plane_instance = setup()

    # Test plane by giving it different slopes in many directions
    plane_instance.test()