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

global encoderPos
encoderPos = 0



class Winch:
    def __init__(self):
        self.pwm = GPIO.PWM(pwmPin, 100)
        self.pwm.start(100)
        self.flag = 0        
        self.lastencPinA = 0
        self.encoderPos = 0
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
    '''
    def encoderA(self, channel):
        GPIO.remove.event.detect(encPinA)
        if GPIO.input(encPinA) == GPIO.input(encPinB):
            if GPIO.input(encPinA) != self.lastencPinA:
                self.encoderPos += 1
            else:
                self.encoderPos -= 1
        self.lastencPinA = GPIO.input(encPinA)
    '''
    def encoderA(self, channel):
        print("FUNCTION")
        if GPIO.input(encPinA) == GPIO.input(encPinB):
            self.encoderPos += 1
        else:
            self.encoderPos -= 1

    '''
    def encoderB(channel):
        GPIO.remove.event.detect(encPinB)
        if GPIO.input(encPinA) == GPIO.input(encPinB):
            encoderPos -= 1
        else:
            encoderPos += 1
    '''
    def motor_run(self):

            self.pwmValue=100
            self.pwm.ChangeDutyCycle(self.pwmValue)
            GPIO.remove_event_detect(encPinA)
            GPIO.wait_for_edge(encPinA, GPIO.BOTH)
            GPIO.remove_event_detect(encPinA)      
            GPIO.add_event_detect(encPinA, GPIO.BOTH, callback=self.encoderA)
                
            #GPIO.add_event_detect(encPinB, GPIO.BOTH, callback=encoderB, bouncetime=200)
            print(self.encoderPos)

if __name__=="__main__":
    motor = Winch()
    motor.motor_run()
