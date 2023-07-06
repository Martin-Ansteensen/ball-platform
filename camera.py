import cv2
import numpy as np
import board
import neopixel
import atexit
import math
import logging
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import argparse
import logging
import json
import os


class Camera():
    """ Class for handling camera """
    def __init__(self, pi_camera = True, show_cv2_windows = False):
        # Path to current directory
        self.path = os.path.dirname(os.path.abspath(__file__)) + "/"
        # Open config file
        with open(self.path + "config.json", "r") as f:
            config_data = json.load(f)["camera"]
        # Use PiCamera module or cv2.VideoCapture
        self.use_pi_camera = pi_camera
        # Show image processing in cv2 windows
        self.show_cv2_windows = show_cv2_windows
        # Video varianbes
        self.frame_width = config_data["frame_width"]
        self.frame_height = config_data["frame_height"]
        self.framrate = config_data["framerate"]
        self.camera_rotation = eval(config_data["camera_rotation"])
        # Turn on LED-ring for an even lighting of the disk
        self.leds = neopixel.NeoPixel(board.D10, 24, brightness=1, auto_write=False)
        self.leds.fill((255,255,255))
        self.leds.show()
        # Turn off LEDs when program stops
        atexit.register(self.turn_off_leds)
        
        if self.use_pi_camera:
            logging.info("Using PiCamera module for video")
            # Initialize PiCamera video stream
            self.videostream = PiCamera()
            self.videostream.resolution = (self.frame_width, self.frame_height)
            self.videostream.framerate = self.framrate
            self.rawCapture = PiRGBArray(self.videostream, size=(self.frame_width, self.frame_height))
            # allow the camera to warm up    
            time.sleep(0.1)
        else:
            logging.info("Using cv2.VideoCapture module for video")
            # Begin video stream
            self.videostream = cv2.VideoCapture(0)
            self.videostream.set(3, self.frame_width)
            self.videostream.set(4, self.frame_height)
            # Set framerate?
            


            # Get the first frame
            ret, frame = self.videostream.read()
            # Print the resolution of the image
            logging.info(f"The resolution of the images: {frame.shape[0]}, {frame.shape[1]}") 
            
            # NOTE: do we need this?
            self.static = None
            # Frame for simulation
            self.static = cv2.rotate(frame, self.camera_rotation)

            # Adjust the resolution in case the camera does not support
            # the resolution set in config
            self.frame_height = frame.shape[0]  
            self.frame_width = frame.shape[1]


        # Default value for slider controlling thresholding of image
        self.tresh_lower = config_data["cv2_threshold"]
        # Number of frames retrieved
        self.num_frames = 0
        
        if self.show_cv2_windows:
            logging.info("Will be displaying image processing in cv2 windows")
            # Create slider to threshold image correctly
            cv2.namedWindow("Preprocessed image")
            cv2.createTrackbar("slider", "Preprocessed image", self.tresh_lower,255, self.slider_change)  
        else:
            logging.info("Not displaying image processing in cv2 windows")

        # Dict for storing position-data of disk and ball
        self.objects = {"disk":{"x":None, "y":None, "r":None}, "ball":{"x":None, "y":None, "r":None}}
 

    def turn_off_leds(self):
        """ Turns of LEDs """
        self.leds.fill((0,0,0))
        self.leds.show()

    def slider_change(self, value):
        """ Function called when value of slider changes """
        # Update the value used for thresholding the image
        self.tresh_lower = value
        # Load json file
        with open(self.path + "config.json" "r") as f:
            data = json.load(f)
        # Change deafult value
        data["camera"]["cv2_threshold"] = value
        # Write to file
        with open(self.path + "config.json" "w") as f:
            json.dump(data, f, indent=4)


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
        
        if self.show_cv2_windows:	
            # Display the processed image to the user
            cv2.imshow("Preprocessed image", erode)
            
        self.detected_shapes_coords = []
        # Find contours in the image
        contours, _ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Sort contours based on area from largest to smallest
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        return frame, contours

    def locate_disk(self, frame):
        """ Finds disk in picture """ 
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

            # The first large round object in the list of contours 
            # (picture) must be the disk
            if circle_area/picture_area > 0.3:
                self.objects["disk"] = {"x":int(center[0]), "y":int(center[1]), "r":radius}
                logging.critical("Found disk")
                return

    def find_ball(self,frame):
        """ Locates ball in image. Highlights them."""
        frame, contours = self.find_contours(frame)

        # Loop over each contour
        # Assume that the first circle in the contours list that 
        # is located on the disk is the ball.
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

            # Maybe the ball
            found_ball = False
            # It is not the disk
            if circle_area/picture_area < 0.3:

                # Check that the x-coordinate of the circle is larger than (further to the right) the left most point of the disk's perimeter, 
                # and that the x-coordinate is smaller (further to the left) than the right most point of the disk's perimeter
                a = center[0] > self.objects["disk"]["x"] - self.objects["disk"]["r"] and center[0] < self.objects["disk"]["x"] + self.objects["disk"]["r"]
                # Check that the y-coordinate of the circle is larger than (further down) the highest of the disk's perimeter, 
                # and that the y-coordinate is smaller (further up) than the right lowest point of the disk's perimeter
                b = center[1] > self.objects["disk"]["y"] - self.objects["disk"]["r"] and center[1] < self.objects["disk"]["y"] + self.objects["disk"]["r"]
                # If all of the conditions are met, the circle is on the disk.
                # Assume that this circle is the ball
                if not (a and b):
                    # We have not found the ball, try the next circle
                    continue
            
                # This circle is the ball
                self.objects["ball"] = {"x": int(center[0]), "y":int(center[1])}
                if self.show_cv2_windows:
                    # Higlight the ball
                    cv2.circle(frame, (self.objects["ball"]["x"], self.objects["ball"]["y"]), int(radius), (0, 0, 255), 10)
                    # Draw a line from the center of the ball to the center of the disk
                    cv2.line(frame, (self.objects["ball"]["x"], self.objects["ball"]["y"]), (self.objects["disk"]["x"], self.objects["disk"]["y"]), (255, 255, 0), 3)
                    
                found_ball = True
                break
            
            if not found_ball:
                # We have not found the ball
                self.objects["ball"] = {"x": None, "y":None}
            
        if self.show_cv2_windows:
            # Draw disk onto image
            cv2.circle(frame, (int(self.objects["disk"]["x"]), int(self.objects["disk"]["y"])), radius=3, color=(0,255,0), thickness=3)
            # Draw the perimeter of the circle onto the image
            cv2.circle(frame, (int(self.objects["disk"]["x"]), int(self.objects["disk"]["y"])), int(self.objects["disk"]["r"]), (255, 0, 0), 2)
            # Show image with all contours and shapes on it
            cv2.imshow("Final", frame)

    def get_positions(self):
        """ Returns last known position of ball and disk"""
        return self.objects

    # NOTE: is this function in use?
    def simulate(self):
        draw = self.static
        self.objects = {"disk":{"x":289, "y":257, "r":210}, "ball":{"x":200, "y":257, "r":20}}
        
        # Draw disk
        cv2.circle(draw, (int(self.objects["disk"]["x"]), int(self.objects["disk"]["y"])), radius=3, color=(0,255,0), thickness=3)
        # Draw the perimeter of the circle onto the image
        cv2.circle(draw, (int(self.objects["disk"]["x"]), int(self.objects["disk"]["y"])), int(self.objects["disk"]["r"]), (255, 0, 0), 2)

        # Higlight the ball
        cv2.circle(draw, (self.objects["ball"]["x"], self.objects["ball"]["y"]), self.objects["ball"]["r"], (0, 0, 255), 10)
        # Draw a line from the center of the ball to the center of the disk
        cv2.line(draw, (self.objects["ball"]["x"], self.objects["ball"]["y"]), (self.objects["disk"]["x"], self.objects["disk"]["y"]), (255, 255, 0), 3)
        cv2.imshow("Final", draw)
        key = cv2.waitKey(1) & 0xFF 

    def pi_camera_next_frame(self, find_and_correct_ball):
        """ Runs forever as loop. Gets the next frame, proccesses it
		and checks if the user wants to quit. Then does the
        funtion provided."""
        for frame in self.videostream.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):

            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            # Find the disk
            if not self.objects["disk"]["x"]:
                logging.warning("Did not find disk")
                self.locate_disk(image)
            else:
                self.find_ball(image)
                if find_and_correct_ball:
                    find_and_correct_ball()
            
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break


    def cv2_videocapture_next_frame(self):
        """ Gets the next frame, proccesses it
		and checks if the user wants to quit or
		save the frame """
        # Get the next frame
        ret, frame = self.videostream.read()
        # Locates  and highlighrs ball and disk in image

        if not self.objects["disk"]["x"]:
            logging.warning("Did not find disk")
            self.locate_disk(frame)

        else:
            self.find_ball(frame)
        
		# Get the status of the keyboard keys
        key = cv2.waitKey(1) & 0xFF 
		# Exit the program if the user presses "q" or "x"
        if key == ord("q") or key == ord("x"): 
			# Cleanup before exit.
            cv2.destroyAllWindows()
            exit()
		

if __name__=='__main__':
    logging.basicConfig(level=logging.INFO)

    # Arguments from command line
    parser = argparse.ArgumentParser()
    # Argument for using the PiCamera module
    parser.add_argument("--picam", action="store_true", help="Add if you want to use the PiCamera module")
    parser.add_argument("--no-picam", dest="picam", action="store_false", help="Add if you want to use standard cv2.VideoCapture module")
    parser.set_defaults(picam=True)
    # Argument for showing image processing windows
    parser.add_argument("--show", action="store_true", help="Add if you want to display cv2 windows showing the process")
    parser.add_argument("--no-show", dest="show", action="store_false", help="Add if you do not want to display cv2 windows")
    parser.set_defaults(show=False)
    # Get arguments passed through command line
    args = parser.parse_args()

    cam = Camera(args.picam, args.show)

    if cam.use_pi_camera:
        cam.pi_camera_next_frame(None)
    else:
        while True:
            cam.cv2_videocapture_next_frame()
