#!/usr/bin/env python
import RPi.GPIO as IO
import time
import rospy
from std_msgs.msg import Int32, Float32
from math import pi

class motor_control:
    def encoderA(self, channel):
        if IO.input(self.encPinA) == IO.input(self.encPinB):
            self.encoderPos += 1
        else:
            self.encoderPos -= 1
        #print('PinA : %d, encoder : %d' %(channel, self.encoderPos))
    
    def encoderB(self, channel):
        if IO.input(self.encPinA) == IO.input(self.encPinB):
            self.encoderPos -= 1
        else:
            self.encoderPos += 1
        #print('PinB : %d, encoder : %d' %(channel, self.encoderPos))

    def __init__(self):
        self.encPinA = 23
        self.encPinB = 24
        self.pwmPin = 19
        self.dirPin1 = 13 
        self.dirPin2 = 6
        IO.setmode(IO.BCM)
        IO.setwarnings(False)
        IO.setup(self.pwmPin, IO.OUT)
        IO.setup(self.dirPin1, IO.OUT)
        IO.setup(self.dirPin2, IO.OUT)
        IO.setup(self.encPinA, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.encPinB, IO.IN, pull_up_down=IO.PUD_UP)
        self.encoderPos = 0
        IO.add_event_detect(self.encPinA, IO.RISING, callback=self.encoderA)
        IO.add_event_detect(self.encPinB, IO.RISING, callback=self.encoderB)
        self.pwm = IO.PWM(self.pwmPin, 100)
        self.output = 0 # pwm output has to be made
        self.target = 230
        self.error_I = 0
        self.P_term = 100
        self.dt = 0.1
        self.I_term = 0
        self.winch = -1
        self.encoder_pub = rospy.Publisher('/encoder',Float32, queue_size = 10)
        rospy.Subscriber('/winch_roll', Int32, self.missionCb)

    def missionCb(self, msg):
        self.winch = msg.data

    def motor_run(self):
        self.pwm.start(0)
        if self.winch == 10:
            IO.output(self.dirPin1, 0)
            IO.output(self.dirPin2, 1)
            self.pwm.ChangeDutyCycle(100)

        if self.winch == 0:
            self.pwm.ChangeDutyCycle(0)

        if self.winch == -10:
            IO.output(self.dirPin1, 1)
            IO.output(self.dirPin2, 0)
            self.pwm.ChangeDutyCycle(100)
        self.publish(self.encoderPos/(26.0*50.0)*1.1*2*pi*3.0)

    def pid(self):
        encoder_value = self.encoderPos/(26.0*50.0)*1.1*2*pi*3.0
        self.publish(encoder_value)
        error = abs(self.target - encoder_value)/30
        if error > 1:
            error = 1
        #self.error_I += error*self.dt*self.I_term
        self.output = error*self.P_term + self.error_I
        #print(self.encoderPos)
        return self.output        

    def publish(self, encoder_pose):
        encoder_value = Float32()
        encoder_value.data = encoder_pose # publishing raw encoder value 
        print(encoder_value)
        self.encoder_pub.publish(encoder_value)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('winch', anonymous=True, disable_signals=True)
    motor = motor_control()

    try:
        while not rospy.is_shutdown():
            motor.motor_run()
    except rospy.ROSInterruptException:
        motor.pwm.stop()
        pass
