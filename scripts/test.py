#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


current_state = State()
offb_set_mode = SetMode()

def stateCb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscriber
rospy.Subscriber('/mavros/state', State, stateCb)

        # Service_client
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

pose = PoseStamped()

pose.pose.position.x = 0
pose.pose.position.x = 0
pose.pose.position.x = 2

def control():
    rospy.init_node('offb', anonymous=True)

    rate = rospy.Rate(20.0)

    while not current_state.connected:
        rate.sleep()
    
    lsat_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.get_rostime() - lsat_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            lsat_request = rospy.get_rostime()

        if not current_state.armed and (rospy.get_rostime() - lsat_request > rospy.Duration(5.)):
            arming_client(True)
            lsat_request = rospy.get_rostime()

        if current_state.armed:
            rospy.loginfo_once('armed')
        if current_state.mode == "OFFBOARD":
            rospy.loginfo_once('off')

        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        control()

    except rospy.ROSInterruptException:
        pass