import cv2
import numpy as np
import imutils
import board
import neopixel
import random as rng
import atexit
import math
import logging
import numpy as np
rng.seed(12345)


class Camera():
    """ Class for handling camera """
    def __init__(self):
        self.frame_height = None
        self.frame_width = None
        self.framrate = 60
        self.camera_rotation = None
        # Turn on LED-ring for an even lighting
        self.leds = neopixel.NeoPixel(board.D10, 24, brightness=1, auto_write=False)
        self.leds.fill((255,255,255))
        self.leds.show()
        # Turn off LEDs when program stops
        atexit.register(self.turn_off_leds)

        # Begin video stream
        self.videostream = cv2.VideoCapture(0)
        self.videostream.set(3, self.frame_width)
        self.videostream.set(4, self.frame_height)
        self.videostream.set(4, self.framrate)
        # Get the first frame
        ret, frame = self.videostream.read()
        print("The resolution of the images: ", frame.shape[0], frame.shape[1]) # Print the resolution of the image
		# Adjust the resolution in case the camera does not support
		# the resolution set in config
        self.frame_height = frame.shape[0]  
        self.frame_width = frame.shape[1]

        # Create slider to threshold image correctly
        self.tresh_lower = 83
        cv2.namedWindow("Erode")
        cv2.createTrackbar("slider", "Erode", self.tresh_lower,255, self.slider_change)   


    def turn_off_leds(self):
        """ Turns of LEDs """
        self.leds.fill((0,0,0))
        self.leds.show()


    def slider_change(self, value):
        self.tresh_lower = value

    def cnt_process(self,frame):
        
        # Convert image into grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Set threshold of gray image
        _, threshold = cv2.threshold(gray, self.tresh_lower, 255, cv2.THRESH_BINARY)
        # Remove noise from the image
        kernel = np.ones((5,5), np.uint8)
        dilate = cv2.dilate(threshold, kernel, iterations=2)
        erode = cv2.erode(dilate, kernel, iterations=3)		
        # Display the processed image to the user
        cv2.imshow("Erode", erode)

        self.detected_shapes_coords = []
        contours, _ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        i = 0
        circles = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            # Find center and raduius of circles
            center, radius = cv2.minEnclosingCircle(approx)
            picture_area = self.frame_width*self.frame_height
            circle_area = math.pi*radius*radius
            # Check if it is a circle
            if not cv2.isContourConvex(approx):
                # Not a circle
                # cv2.drawContours(frame, [contour], 0, (0, 255, 255), 3)
                continue

            # If the circle is too small, do not 
            # do the rest of the loop for this item
            # and mark it with red
            if circle_area/picture_area < 0.001:
                # Draw the contours onto the picture
                # cv2.drawContours(frame, [contour], 0, (0, 0, 255), 3)
                logging.debug("Too small")
                continue
            else: 
                # Draw the contours onto the picture
                # cv2.drawContours(frame, [contour], 0, (0, 255, 0), 3)
                logging.debug("Big enough")
            
            # Draw center
            cv2.circle(frame, (int(center[0]), int(center[1])), radius=3, color=(0,255,0), thickness=3)
            # Draw arc
            cv2.circle(frame, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)

            if i > 0 and circles:

                a = center[0] > circles[0][0] - circles[0][2] or center[0] < circles[0][0] + circles[0][2]
                b = center[1] > circles[0][1] - circles[0][2] or center[1] < circles[0][1] + circles[0][2]
                if a and b:
                    # This is the ball
                    ball_center =  (int(center[0]), int(center[1]))
                    plate_center = (int(circles[0][0]), int(circles[0][1]))
                    cv2.circle(frame, ball_center, int(radius), (0, 0, 255), 10)
                    cv2.line(frame,ball_center, plate_center, (255, 255, 0), 3 )
                    
                    error = math.sqrt((center[0]-circles[0][0])**2 + (center[1]-circles[0][1])**2)
                    if error == 0:
                        continue
                    #print(error)
                    # Find mirror vector
                    def dot_vector(a, b):
                        return a[0]*b[0] + a[1]*b[1]
                    # normal_vec = np.array((0,plate_center[1]))
                    
                    line_vec = np.array(ball_center) - np.array(plate_center)
                    if line_vec[1] == 0:
                        line_vec[1] = 1
                    normal_vec = np.array((-1, line_vec[0]/line_vec[1]))

                    scalar = 2*dot_vector(normal_vec, line_vec)/dot_vector(normal_vec,normal_vec)
                    mirror_vec = scalar*normal_vec-line_vec
                    picture_vec = np.array(plate_center) + mirror_vec
                    picture_line = picture_vec.tolist()
                    picture_line[0], picture_line[1] = int(picture_line[0]), int(picture_line[1])
                    cv2.line(frame,plate_center, picture_line,  (0, 255, 255), 3 )
                    z_adjust = error*0.1
                    plane_vec = [picture_line[0], picture_line[1], -z_adjust]
                    nort_plane_vec = [normal_vec[0], normal_vec[1], z_adjust]



            circles.append([int(center[0]), int(center[1]), radius])
            i +=1
        cv2.imshow("final", frame)

    def website_cnt(self, frame):
        threshold = 200
        src_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        src_gray = cv2.blur(src_gray, (3,3))
        canny_output = cv2.Canny(src_gray, threshold, threshold * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    
        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)
        for i, c in enumerate(contours):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])


        drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)


        for i in range(len(contours)):
            color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
            cv2.drawContours(drawing, contours_poly, i, color)
            # cv2.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
            #     (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
            cv2.circle(drawing, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)


        cv2.imshow('Contours', drawing)



    def get_next_frame(self):
        """ Gets the next frame, proccesses it
		and checks if the user wants to quit or
		save the frame """
        # Get the next frame
        ret, frame = self.videostream.read()
        # Process the frame
        self.cnt_process(frame)
		# Get the status of the keyboard keys
        key = cv2.waitKey(1) & 0xFF 
		# # Exit the program if the user presses 'x'
        # if key == ord("q"): 
		# 	# Cleanup before exit.
        #     cv2.destroyAllWindows()
        #     exit()
		
		# # Save the image if the user presses 's'
        # if key == ord("s"):
        #     self.saveImage()

if __name__=='__main__':
    cam = Camera()

    while True:
        cam.get_next_frame()