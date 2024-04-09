#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import numpy
import statistics
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import math
import random
import subprocess


trigPin = 23 #GPIO23

echoPin = 24

MAX_DISTANCE = 500

timeOut = MAX_DISTANCE*60

trackingPinLeft = 5
trackingPinRight = 6

buttonPin = 20

i2c = busio.I2C(SCL, SDA)


#Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

pca.frequency = 50 # the 20 ms width required by these servos

#servo0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400)#SG90
servoLeft = servo.Servo(pca.channels[2], min_pulse=900, max_pulse=2100)#HS422
servoRight = servo.Servo(pca.channels[6], min_pulse=900, max_pulse=2100)#HS422

def pulseIn(pin, level, timeOut): #obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if (( time.time() - t0) > timeOut*0.000001):
            return 0
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time()-t0) > timeOut * 0.000001):
            return 0
    pulseTime = (time.time() - t0)* 1000000
    return pulseTime

def getSonar(): #get the measurement results of ultrasonic module, with unit: cm
    GPIO.output(trigPin,GPIO.LOW)  #make trigPin output 10 microseconds HIGH level
    time.sleep(0.000002)

    GPIO.output(trigPin,GPIO.HIGH)  #make trigPin output 10 microseconds HIGH level
    time.sleep(0.00001)
    GPIO.output(trigPin, GPIO.LOW)  #make trigPin output LOW level
    pingTime = pulseIn(echoPin,GPIO.HIGH, timeOut)  #read plus time of echoPin
    distance = pingTime * 340.0 / 2.0 / 10000.0 # calculate distance with sound speed 340m/s and pingTime in microseconds
    return distance

#This function gets 10 readings from the sonic range finder and then trims out any of the bad readings
#It will return a list of final distances, the variance of the distances, and the standard deviation
def getDistance(t):
    distances = []
    for i in range(10):
        distances.append(getSonar())
        if t != 0:
            time.sleep(t)


    elements = numpy.array(distances)

    #average
    mean = numpy.mean(elements)

    #standard deviation
    sd = numpy.std(elements)

    #Variance
    v = numpy.var(elements)

    #This strips out any bad distances
    final = [x for x in distances if (x > mean - 2 * sd)]
    final = [x for x in final if (x < mean + 2 * sd)]
    
    return final, v, sd
    
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trackingPinLeft, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(trackingPinRight, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(trigPin, GPIO.OUT)   #set trigPin to OUTPUT mode
    GPIO.setup(echoPin, GPIO.IN)    #set echoPin to INPUT mode


def loop():
    changeMotors(-0.1, 0.1)
    while True:
        if GPIO.input(buttonPin) == GPIO.LOW:
            subprocess.Popen(['shutdown','-h','now'])
        
        trackLines()

def changeMotors(left, right):
    servoLeft.angle = convertLeftSpeed(left)
    servoRight.angle = convertRightSpeed(right)


def trackLines():
    while True:


        dist, v, sd = getDistance(0)
        dist = numpy.mean(dist)

        if dist < 25:
            changeMotors(-0.50 ,0.50)

        elif dist >= 25:
            changeMotors(-0.1, 0.1)

        if GPIO.input(trackingPinLeft) == GPIO.LOW:
            #Print this is when they detect something black
            backUpLeft()

        if GPIO.input(trackingPinRight) == GPIO.LOW:
            #Print this is when they detect something black
            backUpRight()

def backUpLeft():
    times = [1.25, 1.5, 1.75]
    r = random.randint(0,2)
    print("BACK UP LEFT")
    changeMotors(0.5, -0.5)
    time.sleep(.75)
    changeMotors(-0.75, -0.75)
    time.sleep(times[r])
    changeMotors(-0.1, 0.1)


def backUpRight():
    print("BACK UP RIGHT")
    times = [1.25, 1.5, 1.75]
    r = random.randint(0,2)
    changeMotors(0.5, -0.5)
    time.sleep(.75)
    changeMotors(0.75, 0.75)
    time.sleep(times[r])
    changeMotors(-0.1, 0.1)

def convertLeftSpeed(speed):
    return( (speed) + 1) * 90

def convertRightSpeed(speed):
    return (speed +1) * 90

def destroy():

    servoLeft.angle = None
    servoRight.angle = None
    pca.deinit()
    GPIO.cleanup()

if __name__ == '__main__':
    setup()

    while True:

        try:
            if GPIO.input(buttonPin) == GPIO.LOW:
                time.sleep(3)
                loop()

        except KeyboardInterrupt:
            destroy()
