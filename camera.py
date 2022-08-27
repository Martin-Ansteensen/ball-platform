import cv2
import numpy as np
import imutils
import board
import neopixel
import atexit
import math
import logging
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import mechanics as mech

import logging
logging.basicConfig(level=logging.ERROR)


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



SHOW_CV2_WINDOW = False
PI_CAMERA = True

class Camera():
    """ Class for handling camera """
    def __init__(self):
        # print(cv2.useOptimized())
        self.frame_height = 480
        self.frame_width = 640
        self.framrate = 30
        self.camera_rotation = cv2.ROTATE_90_COUNTERCLOCKWISE
        # Turn on LED-ring for an even lighting
        self.leds = neopixel.NeoPixel(board.D10, 24, brightness=1, auto_write=False)
        self.leds.fill((255,255,255))
        self.leds.show()
        # Turn off LEDs when program stops
        atexit.register(self.turn_off_leds)
        if PI_CAMERA:
            self.videostream = PiCamera()
            self.videostream.resolution = (self.frame_width, self.frame_height)
            self.videostream.framerate = 32
            self.rawCapture = PiRGBArray(self.videostream, size=(self.frame_width, self.frame_height))
            # allow the camera to warmup    
            time.sleep(0.1)
        else:
            # Begin video stream
            self.videostream = cv2.VideoCapture(0)
            self.videostream.set(3, self.frame_width)
            self.videostream.set(4, self.frame_height)
            self.videostream.set(cv2.CAP_PROP_FPS, self.framrate)
            
            self.static = None

            # Get the first frame
            ret, frame = self.videostream.read()
            print("The resolution of the images: ", frame.shape[0], frame.shape[1]) # Print the resolution of the image
            
            # Frame for simulation
            self.static = cv2.rotate(frame, self.camera_rotation)

            # Adjust the resolution in case the camera does not support
            # the resolution set in config
            self.frame_height = frame.shape[0]  
            self.frame_width = frame.shape[1]


        # Default value for slider
        self.tresh_lower = 69
        
        if SHOW_CV2_WINDOW:
            # Create slider to threshold image correctly
            cv2.namedWindow("Preprocessed image")
            cv2.createTrackbar("slider", "Preprocessed image", self.tresh_lower,255, self.slider_change)  

        # Dict for storing position-data of plate and ball
        self.objects = {"plate":{"x":None, "y":None, "r":None}, "ball":{"x":None, "y":None, "r":None}}
 

    def turn_off_leds(self):
        """ Turns of LEDs """
        self.leds.fill((0,0,0))
        self.leds.show()

    def slider_change(self, value):
        """ Function called when value of slider changes """
        # Update the value used for thresholding the image
        self.tresh_lower = value

    def find_contours(self, frame):
        """Finds contours in image"""
        # Rotate image
        frame = cv2.rotate(frame, self.camera_rotation)

        # Convert image into grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Threshold imgae
        _, threshold = cv2.threshold(gray, self.tresh_lower, 255, cv2.THRESH_BINARY)
        # Remove noise from the image
        kernel = np.ones((5,5), np.uint8)
        erode = cv2.erode(threshold, kernel, iterations=1)
        
        if SHOW_CV2_WINDOW:	
            # Display the processed image to the user
            cv2.imshow("Preprocessed image", erode)
            
        self.detected_shapes_coords = []
        # Find contours in the image
        contours, _ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Sort contours based on area from largest to smallest
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        return frame, contours

    def locate_plate(self, frame):
        """Finds plate in picture""" 
        frame, contours = self.find_contours(frame)
        for (i, contour) in enumerate(contours):
            
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # We are only looking for round objects.
            # If the contour is not circular, do not
            # perform further calculations on the contour
            if not cv2.isContourConvex(approx):
                # The shape is not a circle
                # Mark the shape in the image 
                # cv2.drawContours(frame, [contour], 0, (0, 255, 255), 3)
                continue
            
            # Approximate center and raduius of circle
            center, radius = cv2.minEnclosingCircle(approx)
            # Calculate the area of the image (from the camera)
            picture_area = self.frame_width*self.frame_height
            # Calculate the area of the circle
            circle_area = math.pi*radius*radius

            # The first large round object in the picture must be the plate
            if circle_area/picture_area > 0.3:
                # Assume that the first circle in the contours 
                # list is the plate (it will be the largest round object).
                self.objects["plate"] = {"x":int(center[0]), "y":int(center[1]), "r":radius+10}
                return

    def find_positions(self,frame):
        """ Locates ball and plate in image. Highlights them."""
        frame, contours = self.find_contours(frame)

        # Loop over each contour
        # Assume that the first circle in the contours list that is located on the plate is the ball.
        for (i, contour) in enumerate(contours):
            
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # We are only looking for round objects.
            # If the contour is not circular, do not
            # perform further calculations on the contour
            if not cv2.isContourConvex(approx):
                # The shape is not a circle
                # Mark the shape in the image 
                # cv2.drawContours(frame, [contour], 0, (0, 255, 255), 3)
                continue
            
            # Approximate center and raduius of circle
            center, radius = cv2.minEnclosingCircle(approx)
            # Calculate the area of the image (from the camera)
            picture_area = self.frame_width*self.frame_height
            # Calculate the area of the circle
            circle_area = math.pi*radius*radius

            # If the circle is too small, do not 
            # do the rest of the loop for this item
            if circle_area/picture_area < 0.001:
                # Mark the shape in the image 
                # cv2.drawContours(frame, [contour], 0, (0, 0, 255), 3)
                continue
            else: 
                # Mark the shape in the image 
                # cv2.drawContours(frame, [contour], 0, (0, 255, 0), 3)
                pass

            # if SHOW_CV2_WINDOW:
            #     # Draw center of the circle onto the image
            #     cv2.circle(frame, (int(center[0]), int(center[1])), radius=3, color=(0,255,0), thickness=3)
            #     # Draw the perimeter of the circle onto the image
            #     cv2.circle(frame, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)

            # Maybe the ball
            found_ball = False
            # It is not the plate
            if circle_area/picture_area < 0.3:

                # Check that the x-coordinate of the circle is larger than (further to the right) the left most point of the circle's perimeter, 
                # and that the x-coordinate is smaller (further to the left) than the right most point of the circle's perimeter
                a = center[0] > self.objects["plate"]["x"] - self.objects["plate"]["r"] and center[0] < self.objects["plate"]["x"] + self.objects["plate"]["r"]
                # Check that the y-coordinate of the circle is larger than (further down) the highest of the circle's perimeter, 
                # and that the y-coordinate is smaller (further up) than the right lowest point of the circle's perimeter
                b = center[1] > self.objects["plate"]["y"] - self.objects["plate"]["r"] and center[1] < self.objects["plate"]["y"] + self.objects["plate"]["r"]
                # If all of the conditions are met, the circle is on the plate.
                # Assume that this circle is the ball
                if not a and b:
                    # We have not found the ball, try the next circle
                    continue
            
                # This circle is the ball
                self.objects["ball"] = {"x": int(center[0]), "y":int(center[1])}
                if SHOW_CV2_WINDOW:
                    # Higlight the ball
                    cv2.circle(frame, (self.objects["ball"]["x"], self.objects["ball"]["y"]), int(radius), (0, 0, 255), 10)
                    # Draw a line from the center of the ball to the center of the plate
                    cv2.line(frame, (self.objects["ball"]["x"], self.objects["ball"]["y"]), (self.objects["plate"]["x"], self.objects["plate"]["y"]), (255, 255, 0), 3)
                    
                found_ball = True
                break
            
            if not found_ball:
                # We have not found the ball
                self.objects["ball"] = {"x": None, "y":None}
            
        if SHOW_CV2_WINDOW:
            # Draw plate onto image
            cv2.circle(frame, (int(self.objects["plate"]["x"]), int(self.objects["plate"]["y"])), radius=3, color=(0,255,0), thickness=3)
            # Draw the perimeter of the circle onto the image
            cv2.circle(frame, (int(self.objects["plate"]["x"]), int(self.objects["plate"]["y"])), int(self.objects["plate"]["r"]), (255, 0, 0), 2)
            # Show image with all contours and shapes on it
            cv2.imshow("Final", frame)

    def get_positions(self):
        """ Returns last known position of ball and plate"""
        return self.objects

    def simulate(self):
        draw = self.static
        self.objects = {"plate":{"x":289, "y":257, "r":210}, "ball":{"x":200, "y":257, "r":20}}
        
        # Draw plate
        cv2.circle(draw, (int(self.objects["plate"]["x"]), int(self.objects["plate"]["y"])), radius=3, color=(0,255,0), thickness=3)
        # Draw the perimeter of the circle onto the image
        cv2.circle(draw, (int(self.objects["plate"]["x"]), int(self.objects["plate"]["y"])), int(self.objects["plate"]["r"]), (255, 0, 0), 2)

        # Higlight the ball
        cv2.circle(draw, (self.objects["ball"]["x"], self.objects["ball"]["y"]), self.objects["ball"]["r"], (0, 0, 255), 10)
        # Draw a line from the center of the ball to the center of the plate
        cv2.line(draw, (self.objects["ball"]["x"], self.objects["ball"]["y"]), (self.objects["plate"]["x"], self.objects["plate"]["y"]), (255, 255, 0), 3)
        cv2.imshow("Final", draw)
        key = cv2.waitKey(1) & 0xFF 

    def piCameraNextFrame(self):
        i = 0
        for frame in self.videostream.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):

            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            if i == 2:
                self.locate_plate(image)
            if i > 2:
                self.find_positions(image)
            
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

            pos_data = cam.get_positions()
            # Correct the position of the ball
            #plane.correct_ball(pos_data)
            plane.two_axis_correct_ball(pos_data)
            i += 1



    def next_frame(self):
        """ Gets the next frame, proccesses it
		and checks if the user wants to quit or
		save the frame """
        # Get the next frame
        ret, frame = self.videostream.read()
        # Locates  and highlighrs ball and plate in image
        self.find_positions(frame)
		# Get the status of the keyboard keys
        key = cv2.waitKey(1) & 0xFF 
		# Exit the program if the user presses "q" or "x"
        if key == ord("q") or key == ord("x"): 
			# Cleanup before exit.
            cv2.destroyAllWindows()
            exit()
		

if __name__=='__main__':
    cam = Camera()

    if PI_CAMERA:
        cam.piCameraNextFrame()
    else:
        while True:
            cam.next_frame()
