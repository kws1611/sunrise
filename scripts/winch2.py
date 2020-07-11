#!/usr/bin/env python
#import rospy
#from sunrise.msg import WayPoint
#from geometry_msgs.msg import Twist
from math import sqrt
from RPi import GPIO
import time

pwmPin = 19
dirPin1 = 13
dirPin2 = 6
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwmPin, GPIO.OUT)
GPIO.setup(dirPin1, GPIO.OUT)
GPIO.setup(dirPin2, GPIO.OUT)
clock = 0
GPIO.output(dirPin1, 1)
GPIO.output(dirPin2, 0)

#encoder

encPinA = 23
encPinB = 24
GPIO.setup(encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)


encoderPos = 0

def encoderA(channel):
    global encoderPos
    global encPinA
    global encPinB
    GPIO.remove_event_detect(encPinA)
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPos += 1
    else:
        encoderPos -= 1
   
def encoderB(channel):
    global encoderPos
    global encPinA
    global encPinB
    GPIO.remove_event_detect(encPinB)
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1


if __name__=="__main__":
    pwm = GPIO.PWM(pwmPin, 100)
    pwm.start(100)
    for i in range(0,50):
        if i<40:
            pwmValue=100       
        else:
            pwmValue=0
        pwm.ChangeDutyCycle(pwmValue)
        #GPIO.add_event_detect(encPinA, GPIO.RISING, callback=encoderA)
        #GPIO.add_event_detect(encPinB, GPIO.RISING, callback=encoderB)
        print(encoderPos)
        time.sleep(0.2)
