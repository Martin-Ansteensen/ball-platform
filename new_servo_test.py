import pigpio
from time import sleep
import math

# connect to the 
pi = pigpio.pi()


class MyServo():
    def __init__(self, pin, min_deg, max_deg, min_pulse, max_pulse):
        self.pin = pin
        self.min_pulse = min_pulse
        self.min_deg = min_deg
        self.max_pulse = max_pulse
        self.max_deg = max_deg 
        self.last_pulse = 0

        # Move to horizontal position
        self.moveToDeg(0)

    def moveToDeg(self, deg, wait = False):
        # Convert deg to pulse
        pulse = self.map(deg, self.max_deg, self.min_deg, self.min_pulse, self.max_pulse)
        # Calculate sleep time
        sleep_time = abs(self.last_pulse-pulse)*0.8/1230 #0.8
        pi.set_servo_pulsewidth(self.pin, pulse)
        self.last_pulse = pulse
        
        if wait:
            sleep(sleep_time)
        
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def detach(self):
        pi.set_servo_pulsewidth(self.pin, 0)


x_servo = MyServo(21, -87, 52, 860, 2080)
y_servo = MyServo(20, -87, 56, 2080, 850)
sleep(2)
# Circular motion
while True:
    for i in range(-360, 360, 1):
        r = math.radians(i)
        m = math.sin(r)
        n = math.cos(r)
        deg_m = x_servo.map(m, -1, 1, -60, 47)
        deg_n = x_servo.map(n, -1, 1, -60, 47)

        x_servo.moveToDeg(deg_m)
        y_servo.moveToDeg(deg_n)
        sleep(0.005)
