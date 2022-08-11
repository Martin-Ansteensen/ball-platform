from gpiozero import Servo
from gpiozero import AngularServo
import math
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()


class MyServo1():
    def __init__(self, pin, horizontal_deg, min_deg, max_deg, reverse=False):
        self.pin = pin
        self.horizontal_rad = math.radians(horizontal_deg)
        self.min_rad = math.radians(min_deg)
        self.max_rad = math.radians(max_deg)
        self.reverse_dir = reverse

        print("Horizontal position for servo in radians")
        print(self.horizontal_rad)
        self.servo = Servo(pin, initial_value = 0,  pin_factory=factory)
        sleep(0.25)
        self.goToRad(0)
        sleep(0.25)

    def goToDeg(self, angle_deg):
        self.goToRad(math.radians(angle_deg))

    def goToRad(self, angle_rad):
        if self.reverse_dir:
            new_rad = -1*(angle_rad + self.horizontal_rad)
        else:
            new_rad = angle_rad + self.horizontal_rad
            
        print(new_rad)
        self.servo.value = new_rad


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#y_servo = MyServo(20, 11.5, -60, 47)
#x_servo = MyServo(21, 20, -60, 47, True)
#x_servo.goToDeg(40)
x_servo = Servo(21, initial_value=0,min_pulse_width = 0.87/1000, max_pulse_width = 2.15/1000, pin_factory=factory)
sleep(1)

pulse = map(0, 52, -98.7,0.87/1000,2.15/1000)
val = map(pulse, 0.87/1000, 2.15/1000, -1, 1)
x_servo.value = val
exit()


multiply_with_deg_for_pulse = (0.87/1000+2.15/1000)/(52+98.7)
my_deg = 0

pulse = my_deg*multiply_with_deg_for_pulse

min_deg_in_min_pulse_out = map(-60, -98.7, 52, 0.87/1000,2.15/1000)
max_deg_in_min_pulse_out = map(47, -98.7, 52, 0.87/1000,2.15/1000)



min_deg_output = map(-60, -98.7, 52, 0.87/1000,2.15/1000)
print(min_deg_output)
max_deg_output = map(47, -98.7, 52, 0.87/1000,2.15/1000)

print(max_deg_output)
zero_deg = map(-60, -60, 47, min_deg_output, max_deg_output)

# x_servo.min()
# sleep(1)
# x_servo.max()
# sleep(1)
print(zero_deg)
x_servo.value = zero_deg
sleep(1)


exit()


# y_zero = 18
# x_zero = 33

y_zero = -3
x_zero = -13

# Min and max angle are used to constrict the servo and to
# set its positive direction of rotation. However, I do not use it to
# constrain the servo, because then the AngularServo scales all of the 
# inputs differently (0 deg on scale -90 to 90 is not the same as on scale
# -60 to 47), which makes it harder to tune the zero position of the servo.
y_servo = AngularServo(20, initial_angle = 0, min_angle = -60 + y_zero, max_angle = 47 + y_zero, pin_factory=factory)
sleep(1)

x_servo = AngularServo(21, initial_angle = 0, min_angle = 47 + x_zero, max_angle = -60 + x_zero, pin_factory=factory)
sleep(1000)


x_servo.max()
y_servo.min()
sleep(1)
x_servo.angle = -60
y_servo.angle = -60
sleep(3)


# x_servo.max()
# sleep(1)

# sleep(1)
# y_servo.max()
# sleep(0.5)


# while True:
#     for i in range(-60, 47):
#         y_servo.angle = math.degrees(math.sin(math.radians(i))) + y_zero
#         x_servo.angle = math.degrees(math.cos(math.radians(i))) + x_zero
#         sleep(0.1)

# Detach the servos
y_servo.detach()
x_servo.detach()



"""
y_zero = 0
x_zero = 0
# min_angle=y_zero-60, max_angle=y_zero+47
y_servo = AngularServo(20,pin_factory=factory)
sleep(1)
y_servo.angle = 18
sleep(1)
#min_angle=100, max_angle=-100,
x_servo = AngularServo(21, pin_factory=factory)
sleep(1)
x_servo.angle = -33
sleep(1)

"""


#y_servo.angle = y_zero - 30 

#x_servo.angle = x_zero -20
#sleep(1)

"""
# Go to the min and max positoin
y_servo.value = y_zero + -60
sleep(2)
y_servo.value = y_zero + 47
"""
