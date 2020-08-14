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
    GPIO.remove.event.detect(encPinA)
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPos += 1
    else:
        encoderPos -= 1
   
def encoderB(channel):
    global encoderPos
    GPIO.remove.event.detect(encPinB)
    if GPIO.input(encPinA) == GPIO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1


class Winch:
    def __init__(self):
        self.pwm = GPIO.PWM(pwmPin, 100)
        self.pwm.start(100)
        self.flag = 0
        '''self.velLimit = 0
        self.angVelLimit = 0
        rospy.Subscriber('/sunrise/waypoint', WayPoint, self.waypoint)
        rospy.Subscriber('/mavros/global_position/gp_vel', Twist, self.velocity)

    def waypoint(self, msg):
        self.wp = msg

    def velocity(self, msg):
        x = msg.twist.twist.linear.x
        y = msg.twist.twist.linear.y
        z = msg.twist.twist.linear.z
        a = msg.twist.twist.angular.x
        b = msg.twist.twist.angular.y
        w = msg.twist.twist.angular.z
        self.vel = sqrt(x**2+y**2+z**2)
        self.angVel = sqrt(a**2+b**2+w**2)

    def determine(self):
        if self.wp == 2 && self.vel < self.velLimit && self.angVel < self.angVelLimit :
            self.motor_run()
'''
    def motor_run(self):
        global encoderPos
        for i in range(0,50):
            if i<40:
                self.pwmValue=100
            else:
                self.pwmValue=0
            self.pwm.ChangeDutyCycle(self.pwmValue)
            #GPIO.add_event_detect(encPinA, GPIO.BOTH, callback=encoderA, bouncetime=200)
            #GPIO.add_event_detect(encPinB, GPIO.BOTH, callback=encoderB, bouncetime=200)
            print(encoderPos)
            time.sleep(0.2)

if __name__=="__main__":
    motor = Winch()
    motor.motor_run()
