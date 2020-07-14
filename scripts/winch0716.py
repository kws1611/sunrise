#!/usr/bin/env python
#import rospy
#from sunrise.msg import WayPoint
#from geometry_msgs.msg import Twist
from math import sqrt, pi
from RPi import GPIO
import time
import datetime

class Winch:
    def __init__(self):
        self.encoderPos = 0
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
        self.flag = 0
        self.radius = 0.03
        self.goal = 2
        self.length = 0
        self.prev_time = int(round(time.time()*1000))
        self.now = 0
        self.passed = 0

    def Rott(self, channel):
        print("Event!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", self.encoderPos)
        stateA = GPIO.input(self.encPinA)
        stateB = GPIO.input(self.encPinB)
        print("encoderPos", self.encoderPos)
        
        if (self.passed >= 7300):   # <<<<< Time Edit!(ms)
                print("Rott Func : passed!")
                self.pwmValue=0
                self.pwm.ChangeDutyCycle(self.pwmValue)
                GPIO.remove_event_detect(self.encPinA)

                self.flag = 1
                prev_time = self.now
                return

        if (stateA == 1) and (stateB == 0):
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : 1 & 0")
            print("encoderPos", self.encoderPos)

            while stateB == 0:
                stateB = GPIO.input(self.encPinB)
            while stateB == 1:
                stateB = GPIO.input(self.encPinB)
            
            self.encoderPos += 1
            return

        elif (stateA == 1) and (stateB == 1):
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : 1 & 1")
            print("encoderPos", self.encoderPos)

            while stateA == 1:
                stateA = GPIO.input(self.encPinA)
            self.encoderPos -= 1
            return

        else:
            return


    def motor_run(self):
        self.pwmValue=100
        self.pwm.ChangeDutyCycle(self.pwmValue)        
        GPIO.remove_event_detect(self.encPinA)
        GPIO.add_event_detect(self.encPinA, GPIO.RISING, self.Rott, bouncetime=100)
        while(1):
            #print("Detecting events")
            self.now = int(round(time.time()*1000))
            self.passed = self.now - self.prev_time
            print(self.now)
            print(self.passed)
            time.sleep(0.05)
            if self.flag == 1:
                break
        '''
        self.pwmValue=0
        self.pwm.ChangeDutyCycle(self.pwmValue)
        '''
        return


if __name__=="__main__":
    motor = Winch()
    motor.motor_run()
