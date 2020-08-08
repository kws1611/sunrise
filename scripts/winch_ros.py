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
        IO.setup(self.pwmPin, IO.OUT)
        IO.setup(self.dirPin1, IO.OUT)
        IO.setup(self.dirPin2, IO.OUT)
        IO.setwarnings(False)
        IO.setup(self.encPinA, IO.IN, pull_up_down=IO.PUD_UP)
        IO.setup(self.encPinB, IO.IN, pull_up_down=IO.PUD_UP)
        self.encoderPos = 0
        IO.add_event_detect(self.encPinA, IO.BOTH, callback=self.encoderA)
        IO.add_event_detect(self.encPinB, IO.BOTH, callback=self.encoderB)
        self.output = 0 # pwm output has to be made
        self.target = 230
        self.error_I = 0
        self.P_term = 100
        self.dt = 0.1
        self.I_term = 0
        self.flag = 0
        self.winch = -1
        self.mission_check = False
        self.encoder_pub = rospy.Publisher('/encoder',Float32, queue_size = 10)
        rospy.Subscriber('/winch_roll', Int32, self.missionCb)

    def missionCb(self, msg):
        self.winch = msg
            
    def motor_run(self):
        if self.winch == 10:
            if self.flag == 0:        
                IO.output(dirPin1, 1)
                IO.output(dirPin2, 0)
                self.flag = 1
            self.pwmChangeDutyCycle(self.pid())
        if self.winch == 0:
            self.flag = 0
            self.pwmChangeDutyCycle(0)
        if self.winch == -10:
            if self.flag == 0:
                IO.output(dirPin1, 0)
                IO.output(dirPin2, 1)
                self.flag = 1
                self.target = 0
            self.pwmChangeDutyCycle(self.pid())

    def pid(self):
        self.publish(self.encoderPos/(26.0*50.0)*2*pi*3.0)
        error = abs((self.target - self.encoderPos/(26.0*50)*2*pi*3)/3.0)
        if error > 1:
            error = 10
        #self.error_I += error*self.dt*self.I_term
        self.output = error*self.P_term + self.error_I
        #print(self.encoderPos)
        rospy.sleep(0.1)
        return self.output

    def publish(self, encoder_pose):
        encoder_value = Float32()
        #print(self.encoderPos)
        encoder_value.data = encoder_pose # publishing raw encoder value 
        self.encoder_pub.publish(encoder_value/(26.0*50)*2*pi*3.0)

        
    
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('motor', anonymous=True, disable_signals=True)
    motor = motor_control()

    try:
        
        while not rospy.is_shutdown():
            motor.motor_run()
    except rospy.ROSInterruptException:
        pass