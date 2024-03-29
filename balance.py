import mechanics as mech
import camera
import argparse
import logging

# Do servo setup, and math stuff
plane_instance = mech.setup()


def find_and_correct_ball():
    # Get dict with position data from camera
    pos_data = cam.get_positions()
    # Set target_positon for ball
    plane_instance.set_ball_target_point(pos_data["disk"]) 
    # Correct the position of the ball
    plane_instance.correct_ball(pos_data)

if __name__ == '__main__':
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

    # Initialze camera
    cam = camera.Camera(args.picam, args.show)

    # Start video
    if cam.use_pi_camera:
        cam.pi_camera_next_frame(find_and_correct_ball)
    else:
        while True:
            cam.cv2_videocapture_next_frame()
            find_and_correct_ball()



