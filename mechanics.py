from cmath import sqrt
import math as m
import numpy as np
import pigpio
from time import sleep
import atexit
import logging

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

    def set_servo_angel(self, angel_rad):
        """ Moves the servo to the desired position and updates
        its position variable """

        # Move servo
        self.servo.moveToDeg(calc.to_deg(angel_rad), wait=True)
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

    def update(self):
        """ Updates its x- and y points (moves servo). Updates normal vector """
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
    
    def set_plane_from_angels(self, x_angel_rad, y_angel_rad):
        """ Moves the plane to a certain angel in the XZ-plane
        and the YZ-plane """
        x_servo_rad = self.x_arm.calc_servo_angel(x_angel_rad)
        y_servo_rad = self.y_arm.calc_servo_angel(y_angel_rad)
        
        self.x_arm.set_servo_angel(x_servo_rad)
        self.y_arm.set_servo_angel(y_servo_rad)

        self.update()
    
    def set_plane_from_vec_and_error(self, vec, e):
        """ Moves the plane so that has a slope given by the error, pointing along the vector"""
        pass

    def set_plane_from_vecs(self, u, v):
        " Moves plane so that its normal vector is the cross product of the two given vector"
        # Calc norm vec
        norm_vec = np.cross(u, v)
        print("PLANE VEC", norm_vec.tolist())
        # Get x and y angel of plane
        x_rad = np.arcsin(norm_vec[0]/norm_vec[2])
        y_rad = np.arcsin(norm_vec[1]/norm_vec[2])
        print("x", x_rad*180/m.pi)
        print("y", y_rad*180/m.pi)

        # Set plane according to these angles
        #print(str(x_rad*m.pi/180), str(y_rad*m.pi/180))
        self.set_plane_from_angels(x_rad, y_rad)

    def correct_ball(self, pos_data):
        # Check that we have valid data
        ball_detected = True
        for object in pos_data:
            for i in pos_data[object]:
                if not pos_data[object][i]:
                    ball_detected = False
        
        if not ball_detected:
            self.set_plane_from_angels(0, 0)
            return

        # Calculate the distance from the center of the ball to the center of the plate
        error = m.sqrt((pos_data["ball"]["x"]-pos_data["plate"]["x"])**2 + (pos_data["ball"]["y"]-pos_data["plate"]["y"])**2)
        print(error)
        # Do not do anything if the error is zero
        if error == 0:
            return

        # Scale the error to give a meaningful z-correction
        # Maybe add -1 to get correct correction

        # Implement PID-controller
        z_adjust = (error**2)*0.0015
        #z_adjust = (error**2)*0.0023

        # Find vector from center of ball to center of plate (2D)
        ball_plate_vec = np.array((pos_data["ball"]["x"], pos_data["ball"]["y"])) - np.array((pos_data["plate"]["x"], pos_data["plate"]["y"]))
        # Find 2D vector that is normal to this vector
        normal_vec = np.array((-1*ball_plate_vec[1], ball_plate_vec[0], 0))
        print("N-Vector of R", normal_vec.tolist())
        # Find 3D vector that points in direction we want the ball to move in
        correction_vec = np.array(([ball_plate_vec[0], ball_plate_vec[1], z_adjust]))
        print("C-VECTOR", correction_vec.tolist())

        #print(normal_vec.tolist(), correction_vec.tolist())
        # For the ball to move in the direction of the correction-vec
        # the correction_vec and normal_vec has to lie in the plane
        self.set_plane_from_vecs(normal_vec, correction_vec)


    def test(self):
        """ Runs a series of commands to ensure that 
        the plane is working as intended"""
        logging.info("Slow test")

        logging.info("Moving to horizontal position")
        self.set_plane_from_angels(0, 0)
        sleep(1)
        logging.info("Giving disk -20 deg rotation in x, (plane pointing away from x-servo")
        self.set_plane_from_angels(-20*m.pi/180, 0)
        sleep(1)
        logging.info("Giving disk -20 deg rotation in y, (plane pointing away from x-servo")
        self.set_plane_from_angels(0,-20*m.pi/180)
        sleep(1)
        logging.info("Moving to horizontal position")
        self.set_plane_from_angels(0, 0)

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
        print("Move to horizontal position of the plane")
        self.moveToDeg(0)
        sleep(1)

    def moveToDeg(self, deg, wait = False):
        # Convert deg to pulse
        # pulse = calc.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)
        # Change the zero position from horizontal of plane to
        # horizontal of servo arm
        #pulse_correction = calc.map(self.zero_deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)
        if deg > self.max_deg:
            deg = self.max_deg
        if deg < self.min_deg:
            deg = self.min_deg
            
        pulse_corrected = calc.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)

        # Calculate sleep time
        sleep_time = abs(self.last_pulse-pulse_corrected)*0.8/1230 #0.8
        self.pwm.set_servo_pulsewidth(self.pin, pulse_corrected)
        self.last_pulse = pulse_corrected
        logging.debug("Moving servo to", deg , "measured from when the servo arm is horizontal")
        if wait:
            sleep(sleep_time)
        
    def detach(self):
        self.pwm.set_servo_pulsewidth(self.pin, 0)

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


if __name__=='__main__':
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
    # Test plane by giving it different slopes in many directions
    plane.test()