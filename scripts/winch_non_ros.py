#!/usr/bin/env python
import RPi.GPIO as IO
import time

class motor_control:
    def encoderA(self, channel):
        if IO.input(self.encPinA) == IO.input(self.encPinB):
            self.encoderPos += 1
        else:
            self.encoderPos -= 1
        print('PinA : %d, encoder : %d' %(channel, self.encoderPos))
    
    def encoderB(self, channel):
        if IO.input(self.encPinA) == IO.input(self.encPinB):
            self.encoderPos -= 1
        else:
            self.encoderPos += 1
        print('PinB : %d, encoder : %d' %(channel, self.encoderPos))

    def __init__(self):
        self.encPinA = 23
        self.encPinB = 24
        IO.setmode(IO.BCM)
        IO.setwarnings(False)
        IO.setup(self.encPinA, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.encPinB, IO.IN, pull_up_down=IO.PUD_UP)
        self.encoderPos = 0
        IO.add_event_detect(self.encPinA, IO.BOTH, callback=self.encoderA)
        IO.add_event_detect(self.encPinB, IO.BOTH, callback=self.encoderB)
        self.output = 0 # pwm output has to be made
        self.target = 0
        self.error_I = 0
        self.P_term = 0
        self.dt = 0.1
        self.I_term = 0
        
    def pid(self):
        error = self.target - self.encoderPos
        self.error_I += error*self.dt*self.I_term
        self.output = error*self.P_term + self.error_I
        #print(self.encoderPos)

if __name__ == '__main__':
    # Initialize node

    motor = motor_control()

    try:
        
        while True:
            motor.pid()
    except :
        pass
