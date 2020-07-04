#!/usr/bin/env python
import rospy
from sunrise.msg import WayPoint
from geometry_msgs.msg import Twist
from math import sqrt

class Winch:
    def __init__(self):
        self.velLimit = 0
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

    def motor_run(self):

