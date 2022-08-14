# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import board
import neopixel

# leds = neopixel.NeoPixel(board.D10, 24, brightness=1, auto_write=False)
# leds.fill((255,255,255))
# leds.show()

# # initialize the camera and grab a reference to the raw camera capture
# camera = PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(640, 480))
# # allow the camera to warmup
# time.sleep(0.1)
# # capture frames from the camera
# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
# 	# grab the raw NumPy array representing the image, then initialize the timestamp
# 	# and occupied/unoccupied text
# 	image = frame.array
# 	# show the frame
# 	cv2.imshow("Frame", image)
# 	key = cv2.waitKey(1) & 0xFF
# 	# clear the stream in preparation for the next frame
# 	rawCapture.truncate(0)
# 	# if the `q` key was pressed, break from the loop
# 	if key == ord("q"):
# 		break

import cv2

frame_height = None
frame_width = None
framrate = 30
camera_rotation = 90
# Turn on LED-ring for an even lighting
leds = neopixel.NeoPixel(board.D10, 24, brightness=1, auto_write=False)
leds.fill((255,255,255))
leds.show()

# Begin video stream
videostream = cv2.VideoCapture(0)
videostream.set(3, frame_width)
videostream.set(4, frame_height)
#videostream.set(cv2.CAP_PROP_FPS, framrate)

static = None

# Get the first frame
ret, frame = videostream.read()
print("The resolution of the images: ", frame.shape[0], frame.shape[1]) # Print the resolution of the image

while True:
            # Get the next frame
        ret, frame = videostream.read()
        # Locates  and highlighrs ball and plate in image
        cv2.imshow("Final", frame)
		# Get the status of the keyboard keys
        key = cv2.waitKey(1) & 0xFF 
		# Exit the program if the user presses "q" or "x"
        if key == ord("q") or key == ord("x"): 
			# Cleanup before exit.
            cv2.destroyAllWindows()
            exit()