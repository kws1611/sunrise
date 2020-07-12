#!/usr/bin/env python
#import rospy
#from sunrise.msg import WayPoint
#from geometry_msgs.msg import Twist
from math import sqrt, pi
from RPi import GPIO
import time

class Winch:
    def __init__(self):
        self.pwmPin = 19
        self.dirPin1 = 13
        self.dirPin2 = 6
        self.encPinA = 23
        self.encPinB = 24
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwmPin, GPIO.OUT)
        GPIO.setup(self.dirPin1, GPIO.OUT)
        GPIO.setup(self.dirPin2, GPIO.OUT)
        GPIO.output(self.dirPin1, 0)
        GPIO.output(self.dirPin2, 1)
        GPIO.setup(self.encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.pwm = GPIO.PWM(self.pwmPin, 100)
        self.pwm.start(100)
        self.encoderPos = 0
        self.flag = 0
        self.radius = 0.04
        self.goal = 2
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
        print("encPInValue")
        print(GPIO.input(self.encPinA))
        if GPIO.input(self.encPinA) == 1 and GPIO.input(self.encPinB) == 1:
            print("TT")
            self.encoderPos += 1
        if GPIO.input(self.encPinA) == 1 and GPIO.input(self.encPinB) == 0:
            print("FF")
            self.encoderPos -= 1
        while(1):
            if GPIO.input(self.encPinA) == 0:
                break
        print(self.encoderPos)
        print(self.encoderPos/26)

    def motor_run(self):
        self.pwmValue=100
        self.pwm.ChangeDutyCycle(self.pwmValue)
        GPIO.remove_event_detect(self.encPinA)
        GPIO.add_event_detect(self.encPinA, GPIO.RISING, self.encoderA, bouncetime = 200)
        while(1):
            self.length = self.encoderPos*self.radius*2*pi/26
            print("SS")
            print(abs(self.length))
            print(self.goal)
            if abs(self.length) >= self.goal:
                self.flag = 1
                GPIO.output(self.dirPin1, 1)
                GPIO.output(self.dirPin2, 0)            
                self.pwm.ChangeDutyCycle(self.pwmValue)
            if self.flag == 1 and self.length == 0:
                break
            time.sleep(0.2)
        '''
        while(1):
            print(1)
            GPIO.remove_event_detect(self.encPinA)
            print(GPIO.input(self.encPinA))
            GPIO.add_event_detect(self.encPinA, GPIO.RISING, self.encoderA)
            print(self.encoderPos)
            '''

if __name__=="__main__":
    motor = Winch()
    motor.motor_run()
