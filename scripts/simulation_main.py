#!/usr/bin/env python

import rospy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from sunrise.msg import WayPoint
from std_msgs.msg import Bool, Int32, Float32
from math import sin, cos, sqrt, pi
import numpy as np

class Mission:
    def __init__(self):
        self.current_state = State()
        self.global_home_position = GeoPoint()
        self.current_local_position = Point()
        
        self.earth_radius = None
        self.home_set = False
        
        # Waypoints and Obstacle point
        self.local_home_position = None
        self.wpTakeoff = None

        self.wp1 = None
        self.wp2 = None
        self.wp3 = None

        self.obstacle = None

        self.step = 0
        self.Winch_check = False
        self.Winch_back_check = False
        self.mission_start = False
        self.gripper_check = False
        self.winch_length = 0.0
        self.winch_mission_target_length = 220
        self.winch_back_target_length = 15
        # Publisher
        self.local_pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.waypoint_pub = rospy.Publisher('/sunrise/waypoint', WayPoint, queue_size=10)
        self.winch_pub = rospy.Publisher('/winch_roll', Int32, queue_size=10)
        self.gripper_pub = rospy.Publisher('/gripper_switch', Bool, queue_size=10)

        # Subscriber
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.LocalPositionCb)
        rospy.Subscriber('/gripper_status', Bool, self.gripperCb)
        rospy.Subscriber('/encoder',Float32,self.winchCb)

        # Service_client
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    
    def Gripper_publish(self, msg):
        gripper_msg = Bool()
        gripper_msg.data = msg
        self.gripper_pub.publish(gripper_msg)

    def Winch_publish(self, msg):
        winch_msg = Int32()
        winch_msg.data = msg
        self.winch_pub.publish(winch_msg)

    def winchCb(self,msg):
        self.winch_length = msg.data
        if self.winch_length >self.winch_mission_target_length:
            self.Winch_check = True

        if (self.Winch_check) and (self.winch_length<self.winch_back_target_length):
            self.Winch_back_check = True

    def gripperCb(self, msg):
        self.gripper_check = msg.data
    
    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_throttle(1, "Wait FCU connection")

        rospy.loginfo("FCU connected")

    def check_home_setting(self):
        while not self.home_set:
            rospy.loginfo_throttle(1, "Wait Home position Setting")
        rospy.loginfo("Home position is setted")

    def setEarthRadius(self):
        R_long = 6378137 # unit: meter
        R_short = 6356752 # unit: meter

        lat = self.global_home_position.latitude

        self.earth_radius = sqrt(((R_long * cos(lat * pi/180))**2 + (R_short * sin(lat * pi/180))**2))

    def setMode(self, mode):
        # Custom Mode List: 
        # http://wiki.ros.org/mavros/CustomModes

        rate = rospy.Rate(0.5)
        while True:
            self.PubLocalPosition(self.local_home_position)

            if self.current_state.mode != mode:
                self.set_mode_client(base_mode=0, custom_mode=mode)
            
            else:
                break

            rate.sleep()
                
    def setArm(self):
        rate = rospy.Rate(0.5)
        while True:
            if self.current_state.armed is not True:
                self.arming_client(True)

            else:
                break

            rate.sleep()

    def setWayPoints(self):
        self.local_home_position = Point(0, 0, 0)

        self.wpTakeoff = Point(0, 0, rospy.get_param('/takeoff_alt', 30.0))

        self.wp1 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp1'))
        self.wp2 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp2'))
        self.wp3 = self.ConvertLocalPoint(rospy.get_param('/waypoint/wp3'))
        
        rospy.loginfo('Set Waypoint')

    def setObstaclePoints(self):
        # Z axis means the circle range
        self.obstacle = self.ConvertLocalPoint(rospy.get_param('/obstacle'))

        rospy.loginfo('Set Obstacle')

    def stateCb(self, msg):
        prev_state = self.current_state

        self.current_state = msg
        
        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_state.mode)

        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
        
    def homeCb(self, msg):
        self.home_set = True

        self.global_home_position.latitude = msg.geo.latitude
        self.global_home_position.longitude = msg.geo.longitude

    def LocalPositionCb(self, msg):
        self.current_local_position.x = msg.pose.position.x
        self.current_local_position.y = msg.pose.position.y
        self.current_local_position.z = msg.pose.position.z

    def PubLocalPosition(self, point):
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = point.z

        self.local_pose_pub.publish(pose)

    def PublishVelocity(self, vel):
        self.velocity_pub.publish(vel)

    def PublishWayPoint(self, wp):
        self.waypoint_pub.publish(wp)

    def ConvertLocalPoint(self, global_point):
        local_point = Point()

        home_lat = self.global_home_position.latitude
        home_lon = self.global_home_position.longitude
        
        local_point.x = self.earth_radius * cos(home_lat * pi / 180) * (global_point[1] - home_lon) * pi / 180
        local_point.y = self.earth_radius * (global_point[0] - home_lat) * pi / 180
        local_point.z = global_point[2]

        return local_point

    def waypoint_reach_check(self, wp, reach_range):

        x = self.current_local_position.x
        y = self.current_local_position.y
        z = self.current_local_position.z

        d = sqrt((wp[1].x - x)**2 + (wp[1].y - y)**2 + (wp[1].z - z)**2)

        if d < reach_range :
            return True
        else:
            rospy.loginfo_throttle(1, '%s Process <Distance: %.1f>'%(wp[0], d))
            return False

    def calculate_velocity(self, wp):
        velocity = Twist()

        xy_position = np.array((self.current_local_position.x, self.current_local_position.y))
        xy_target = np.array((wp.x, wp.y))

        z_position = self.current_local_position.z
        z_target = wp.z

        # XY control
        obstacle = np.array((self.obstacle.x, self.obstacle.y))
        R  = self.obstacle.z
        d1, d2 = (10.0, R + 20.0)
        XY_max_vel = 5.0
        
        k1 = XY_max_vel / d1
        k2 = 2*XY_max_vel * (R**2) * R*d2/(d2 - R)
        
        distance_goal = np.linalg.norm((xy_target - xy_position))
        distance_obstacle = np.linalg.norm((obstacle - xy_position))

        # XY control, Attractive term
        if distance_goal <= d1:
            gradient = k1 * (xy_position - xy_target)
        else:
            gradient = k1*d1/distance_goal * (xy_position - xy_target)

        # XY control, Repulsive term
        if distance_obstacle <= d2:
            gradient += k2*(1/d2 - 1/distance_obstacle)/(distance_obstacle**3)*(xy_position - obstacle)

        velocity.linear.x = -gradient[0]
        velocity.linear.y = -gradient[1]

        # Z control
        k3 = 2

        velocity.linear.z = k3 * (z_target - z_position)

        return velocity

    def process(self):
        process = {0:['Takeoff', self.wpTakeoff],
                   1:['WP1', self.wp1],
                   2:['WP2', self.wp2],
                   3:['WP3', self.wp3],
                   4:['Return', self.wpTakeoff],
                   5:['Land', self.local_home_position]}.get(self.step, 'END')

        if (process[0] == 'Takeoff') or (process[0] == 'WP2') or (process[0] == 'WP3'):
            self.PubLocalPosition(process[1])
        
        elif (process[0] == 'WP1') or (process[0] == 'Return'):
            if self.waypoint_reach_check(process, 10) is False:
                velocity = self.calculate_velocity(process[1])
                self.PublishVelocity(velocity)

            else:
                self.PubLocalPosition(process[1])

        elif (process[0] == 'Land'):
            self.setMode("AUTO.LAND")

        else:
            self.PublishWayPoint(self.step)
            rospy.loginfo('Mission Complete')
            rospy.signal_shutdown('Mission Complete')
            quit()

        self.PublishWayPoint(self.step)
        result = self.waypoint_reach_check(process, 0.5)

        if result is True:
            if process[0] == 'WP2':
                if self.gripper_check is True:
                    rospy.loginfo_once('Drop complete')
                    self.Winch_publish(-10)
                    rospy.loginfo_once('Winch going up')
                    rospy.loginfo_throttle(1, self.winch_length)
                    limit = 10
                    if self.winch_length<limit :
                        self.step += 1
                        self.Winch_publish(0)
                    
                elif self.winch_length > 70.0:
                    rospy.loginfo_once('Mission start')
                    self.Winch_publish(0) 
                    self.Gripper_publish(True)
                    self.gripper_check = True
                    
                else :
                    rospy.loginfo_once('Winch going down')
                    rospy.loginfo_throttle(1, self.winch_length)
                    self.Winch_publish(10)    ############## 10 -> line going down , 0 -> wait , -10 -> line back up
            else :
                self.step += 1
            rospy.loginfo_once('Done')

        elif (process[0] == 'Land') and (self.current_state.armed is False):
            rospy.loginfo('Done')
            self.step += 1

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('main', anonymous=True, disable_signals=True)
    
    flight = Mission()

    rate = rospy.Rate(20.0)

    try:
        flight.check_FCU_connection()
        rospy.sleep(1)

        flight.check_home_setting()
        flight.setEarthRadius() 
        rospy.sleep(1)
        
        flight.setWayPoints()
        flight.setObstaclePoints()
        rospy.sleep(1)
        
        # flight.setMode("OFFBOARD")
        # rospy.sleep(1)

        # flight.setArm()

        while not rospy.is_shutdown():
            flight.process()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
