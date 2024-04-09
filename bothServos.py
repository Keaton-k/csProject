#this code requires the PCA9685 module
#sudo pip3 install adafruit-circuitpython-pca9685
#sudo pip3 install adafruit-circuitpython-servokit

import time

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)


#Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

pca.frequency = 50 # the 20 ms width required by these servos

#servo0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400)#SG90
servo4 = servo.ContinuousServo(pca.channels[2], min_pulse=900, max_pulse=2100)#HS422
servo0 = servo.ContinuousServo(pca.channels[6], min_pulse=900, max_pulse=2100)#HS422


def loop(servo, angle):
    if servo == 0:
        servo0.angle = angle

    elif servo == 4:
        servo4.throttle = 1.0
        servo0.throttle = -1.0


def destroy():
    servo0.throttle = 0.0
    servo4.throttle = 0.0
    pca.deinit()

if __name__ == '__main__':
    try:
        while True:
            servo = int(input("What servo should move? "))
            if servo == 0:
                angle = int(input("What angle would you like to rotate to? "))
                if angle > 180:
                    angle = 180
                loop(servo, angle)

            elif servo == 4:
                print("Which direction")
                print("0 - 83 for CCW")
                print("84 for halt")
                print("85 - 180 for CW")
                direction = int(input())
                if direction > 180:
                    direction = 180
                loop(servo, direction)


    except KeyboardInterrupt:
        destroy()
