#!/usr/bin/env python
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

class Winch:
    def __init__(self):
        self.pwm = GPIO.PWM(pwmPin, 100)
        self.pwm.start(100)
        self.flag = 0
        self.encoderPos = 0        
        self.prev_time = int(round(time.time()*1000))
        self.now = 0
        self.passed = 0

    def motor_forward(self):    
        GPIO.output(dirPin1, 1)
        GPIO.output(dirPin2, 0)
        time.sleep(1)
        self.pwmValue=100
        self.pwm.ChangeDutyCycle(self.pwmValue)
        while(1):
            self.now = int(round(time.time()*1000))
            self.passed = self.now - self.prev_time
            if (self.passed >= 7300):   # <<<<< Time Edit!(ms)
                print(self.now)
                print(self.passed)
                time.sleep(0.05)
                self.flag = 1
            if self.flag == 1:
                break
        self.pwmValue=0
        self.pwm.ChangeDutyCycle(self.pwmValue)

    def motor_backward(self):        
        self.passed = 0
        self.flag = 0
        GPIO.output(dirPin1, 0)
        GPIO.output(dirPin2, 1)
        time.sleep(1)  
        self.pwmValue=100
        self.pwm.ChangeDutyCycle(self.pwmValue)
        while(1):
            self.now = int(round(time.time()*1000))
            self.passed = self.now - self.prev_time
            if (self.passed >= 7300):   # <<<<< Time Edit!(ms)
                print(self.now)
                print(self.passed)
                time.sleep(0.05)
                self.flag = 1
            if self.flag == 1:
                break
        self.pwmValue=0
        self.pwm.ChangeDutyCycle(self.pwmValue)
