#!/usr/bin/env python

import rospy
import os
from sensor_msgs.msg import NavSatFix, TimeReference
from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import State
from sunrise.msg import WayPoint

class Log:
    def __init__(self):
        dir_ = os.path.dirname(os.path.abspath( __file__ )) + "/../log/log.txt"
        self.f = open(dir_, 'w')

        self.mode = 'None'
        self.waypoint_data = 'None'
        self.gps_time = 'None'
        self.latitude = 'None'
        self.longitude = 'None'
        self.altitude = 'None'
        
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.GpsCallback)
        rospy.Subscriber('/sunrise/waypoint', WayPoint, self.WayPointCallback)
        rospy.Subscriber('/mavros/state', State, self.ModeCallback)
        rospy.Subscriber('/mavros/time_reference', TimeReference, self.TimeCallback)

    def ModeCallback(self, msg):
        if msg.mode == 'OFFBOARD':
            self.mode = '1'
        else:
            self.mode = '0'
    
    def WayPointCallback(self, msg):
        self.waypoint_data = str(msg.waypoint)

    def TimeCallback(self, msg):
        self.gps_time = str(format(msg.time_ref.secs,".2e"))

    def GpsCallback(self, msg):
        self.latitude = str(msg.latitude)
        self.longitude = str(msg.longitude)
        self.altitude = str(round(msg.altitude,1))

    def WriteLog(self):
        data = " ".join([self.mode, self.waypoint_data, self.gps_time, self.latitude, self.longitude, self.altitude, '\n'])
        self.f.write(data)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('gps_log', anonymous=True)

    log = Log()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        log.WriteLog()

        rate.sleep()

    log.f.close()