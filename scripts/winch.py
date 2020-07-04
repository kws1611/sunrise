#!/usr/bin/env python
import rospy
from sunrise.msg import WayPoint

class Winch:
    def __init__(self):
        rospy.Subscriber('/sunrise/waypoint', WayPoint, self.waypoint)
        rospy.Subscriber('/mavros/state',  State, self.state)

    def waypoint(self, msg):
        self.wp = msg
    def state(self, msg):
        
    def determine(self):
        if self.wp == 2 :
            self.motor_run()

    def motor_run(self):
