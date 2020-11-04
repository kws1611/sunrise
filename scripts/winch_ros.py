#!/usr/bin/env python3

import RPi.GPIO as IO
import time
import rospy
from std_msgs.msg import Int32, Float32
from math import pi
from mavros_msgs.msg import State

class motor_control:
    def __init__(self):
        self.pwmPin = 19
        self.dirPin1 = 13 
        self.dirPin2 = 6
        IO.setmode(IO.BCM)
        IO.setwarnings(False)
        IO.setup(self.pwmPin, IO.OUT)
        IO.setup(self.dirPin1, IO.OUT)
        IO.setup(self.dirPin2, IO.OUT)
        self.sw1 = 0
        self.sw2 = 0
        self.sw3 = 0
        self.ref_time1 = 0
        self.ref_time2 = 0
        self.ref_time3 = 0
        self.encoderPos = 0
        self.pwm = IO.PWM(self.pwmPin, 100)
        self.winch = 0
        self.encoder_pub = rospy.Publisher('/encoder',Float32, queue_size = 10)
        rospy.Subscriber('/winch_roll', Int32, self.missionCb)
        rospy.Subscriber('/mavros/state', State, self.ModeCallback)
        self.pwm.start(0)

    def ModeCallback(self, msg):
        if msg.mode == 'AUTO.LAND' and msg.armed == False:
            quit()

    def missionCb(self, msg):
        self.winch = msg.data

    def motor_run(self):
        if self.winch == 10:
            if(self.sw1 == 0):
                self.sw1 = 1
                self.ref_time1 = time.time()
                
            IO.output(self.dirPin1, 0)
            IO.output(self.dirPin2, 1)
            self.pwm.ChangeDutyCycle(100)
            self.publish(time.time() - self.ref_time1)

        if self.winch == 0:
            if(self.sw1 == 1 and self.sw3 == 0):
                self.sw3 = 1
                self.ref_time3 = time.time()

            IO.output(self.dirPin1, 1)
            IO.output(self.dirPin2, 0)
            self.pwm.ChangeDutyCycle(15)

        if self.winch == -10:
            if(self.sw2 == 0):
                self.sw2 = 1
                self.ref_time2 = time.time()

            IO.output(self.dirPin1, 1)
            IO.output(self.dirPin2, 0)
            self.pwm.ChangeDutyCycle(100)
            self.publish((self.ref_time3 - self.ref_time1) - (time.time() - self.ref_time2))

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

    finally:
        motor.pwm.stop()
        pass
