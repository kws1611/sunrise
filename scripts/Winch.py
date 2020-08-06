#!/usr/bin/env python
from math import sqrt
from RPi import GPIO
import time
import rospy
from sunrise.msg import WayPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

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
        GPIO.output(self.dirPin1, 1)
        GPIO.output(self.dirPin2, 0)
        self.encoderPos = 0
        GPIO.setup(self.encPinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encPinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.encPinA, IO.BOTH, callback=self.encoderA)
        GPIO.add_event_detect(self.encPinB, IO.BOTH, callback=self.encoderB)
        rospy.Subscriber('/sunrise/waypoint', WayPoint, self.waypoint)
        rospy.Subscriber(, , self.gripper)  
        self.winch_pub = rospy.Publisher('/encoder', Int32, queue_size = 10)
        self.winch_state = 0 #0 for initial, 1 for finishing going down, 2 for finishing returning
        self.radius = 0.04
        self.goal = 2

    def waypoint(self, msg):
        self.wp = msg
        #if it arrives at the appropriate position, then start motor move.
    
    def gripper(self, msg):
        self.fallen = msg

    def encoderA(self, channel):
        if GPIO.input(self.encPinA) == GPIO.input(self.encPinB):
            self.encoderPos += 1
        else:
            self.encoderPos -= 1
        #print('PinA : %d, encoder : %d' %(channel, self.encoderPos))
    
    def encoderB(self, channel):
        if GPIO.input(self.encPinA) == GPIO.input(self.encPinB):
            self.encoderPos -= 1
        else:
            self.encoderPos += 1
        #print('PinB : %d, encoder : %d' %(channel, self.encoderPos))

    def motor_run(self):
        if self.wp == 2 and self.fallen == 0:
            GPIO.output(dirPin1, 1)
            GPIO.output(dirPin2, 0)
            time.sleep(1)
            self.pwmValue=100
            self.pwm.ChangeDutyCycle(self.pwmValue)
            if self.encoderPos >= self.goal:
                self.pwmValue=0
                self.pwm.ChangeDutyCycle(self.pwmValue)
                self.winch_state = 1
                self.winch_pub(self.winch_state)
        if self.wp == 2 and self.fallen == 1:
            GPIO.output(dirPin1, 0)
            GPIO.output(dirPin2, 1)
            time.sleep(1)  
            self.pwmValue=100
            self.pwm.ChangeDutyCycle(self.pwmValue)
            if self.encoderPos <= 0:
                self.pwmValue=0
                self.pwm.ChangeDutyCycle(self.pwmValue)
                self.winch_state = 2
                self.winch_pub(self.winch_state)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('winch', anonymous=True, disable_signals=True)
    winch = Winch()
    try:
        while not rospy.is_shutdown():
            winch.motor_run()
            if self.winch_state == 2:
                break
    except rospy.ROSInterruptException:
        pass


