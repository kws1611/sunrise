#!/usr/bin/env python
import RPi.GPIO as IO
import time
import rospy
from std_msgs.msg import Float32, Int32

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
        self.mission_check = False
        self.encoder_pub = rospy.Publisher('/encoder',Float32, queue_size = 10)
        rospy.Subscriber('/winch_roll', Int32, self.missionCb)

    def missionCb(self, msg):
        self.mission_check = msg
        
    def pid(self):
        error = self.target - self.encoderPos
        self.error_I += error*self.dt*self.I_term
        self.output = error*self.P_term + self.error_I
        self.publish(self.encoderPos)
        #print(self.encoderPos)
        rospy.sleep(0.1)
    def publish(self, encoder_pose):
        encoder_value = Float32()
        #print(self.encoderPos)
        encoder_value.data = encoder_pose # publishing raw encoder value 
        self.encoder_pub.publish(encoder_value)
    
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('motor', anonymous=True, disable_signals=True)
    motor = motor_control()

    try:
        
        while not rospy.is_shutdown():
            motor.pid()
    except rospy.ROSInterruptException:
        pass
