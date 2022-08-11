import cv2
import numpy as np
import imutils
import board
import neopixel
import random as rng
import atexit

rng.seed(12345)


class Camera():
    """ Class for handling camera """
    def __init__(self):
        self.frame_height = None
        self.frame_width = None
        self.framrate = None
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

        # 
        self.tresh_lower = 60
        cv2.namedWindow("Erode")
        cv2.createTrackbar("slider", "Erode", self.tresh_lower,255, self.slider_change)   


    def turn_off_leds(self):
        """ Turns of LEDs """
        self.leds.fill((0,0,0))
        self.leds.show()

    def process(self, frame):
        frame = imutils.rotate(frame, angle=self.camera_rotation) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Blur using 3 * 3 kernel.
        gray_blurred = cv2.blur(gray, (3, 3))
        cv2.imshow("gray blurr", gray_blurred)

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(gray_blurred,
                        cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,
                    param2 = 30, minRadius = 1, maxRadius = 40)

        # Draw circles that are detected.
        if detected_circles is not None:

            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))

            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                cv2.circle(frame, (a, b), r, (0, 255, 0), 2)

                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)
                cv2.imshow("Detected Circle", frame)

    def blob_detector(self, frame):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area.
        params.filterByArea = False
        # params.minArea = 1500

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.4

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(frame)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)

    def slider_change(self, value):
        self.tresh_lower = value

    def cnt_process(self,frame):
        
        # Convert image into grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Set threshold of gray image
        _, threshold = cv2.threshold(gray, self.tresh_lower, 255, cv2.THRESH_BINARY)
        # Remove noise from the image
        kernel = np.ones((5,5), np.uint8)
        dilate = cv2.dilate(threshold, kernel, iterations=1)
        erode = cv2.erode(dilate, kernel, iterations=1)		
        # Display the processed image to the user
        cv2.imshow("Erode", erode)

        self.detected_shapes_coords = []
        contours, _ = cv2.findContours(erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours[2:]:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            # Draw the contours onto the picture
            cv2.drawContours(frame, [contour], 0, (0, 0, 255), 5)
            # Fin the center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
                frame = cv2.circle(frame, (x, y), radius=3,color=(0,255,0), thickness=3)
            center, radius = cv2.minEnclosingCircle(approx)
            cv2.circle(frame, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), 2)


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
        #self.blob_detector(frame) 
        self.cnt_process(frame)
        #self.website_cnt(frame)
		# Get the status of the keyboard keys
        key = cv2.waitKey(1) & 0xFF 
		# Exit the program if the user presses 'x'
        if key == ord("q"): 
			# Cleanup before exit.
            cv2.destroyAllWindows()
            exit()
		
		# Save the image if the user presses 's'
        if key == ord("s"):
            self.saveImage()

if __name__=='__main__':
    cam = Camera()

    while True:
        cam.get_next_frame()