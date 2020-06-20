#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint, GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandHome, SetMode, CommandTOL
from math import sin, cos, sqrt, pi

class mission:
    def __init__(self):
        self.current_state = State()
        self.home_position = GeoPoint()
        self.current_position = NavSatFix()
        
        self.service_rate = rospy.Rate(0.5)

        self.diff = 47.0

        # Waypoints and Obstacle point
        self.wpTakeoff = GeoPoint()

        self.wp1 = GeoPoint()
        self.wp2 = GeoPoint()
        self.wp3 = GeoPoint()

        self.obstacle = GeoPoint()

        self.wpReachRange = 0.5  # meter
        self.step = 0

        # Publisher
        self.global_pose_pub = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

        # Subscriber
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.positionCb)

        # Service_client
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        #self.set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        #self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_once("Waiting FCU connection")

        rospy.loginfo("FCU connected")

    '''
    def setHome(self):
        while True:
            if self.current_position.status == 0:   # 0 means GPS Fixed
                response = self.set_home_client(True, 0, self.current_position.latitude, self.current_position.longitude, self.current_position.altitude)
                
                if response.success is True:
                    rospy.loginfo("Set Home position")
                    break

            else:
                rospy.loginfo("GPS is not Fixed")

            self.service_rate.sleep()
    '''

    def setMode(self, mode):
        # Custom Mode List: 
        # http://wiki.ros.org/mavros/CustomModes

        while True:
            self.pub_global_position(self.home_position)

            if self.current_state.mode != mode:
                self.set_mode_client(base_mode=0, custom_mode=mode)
            
            else:
                break

            self.service_rate.sleep()
                
    def setArm(self):
        while True:
            if self.current_state.armed is not True:
                self.arming_client(True)

            else:
                break

            self.service_rate.sleep()

    def setWayPoints(self):
        self.wpTakeoff.latitude = self.home_position.latitude
        self.wpTakeoff.longitude = self.home_position.longitude
        self.wpTakeoff.altitude = rospy.get_param('/takeoff_alt', 30.0)

        self.wp1.latitude = rospy.get_param('/waypoint/wp1')[0]
        self.wp1.longitude = rospy.get_param('/waypoint/wp1')[1]
        self.wp1.altitude = rospy.get_param('/waypoint/wp1')[2]

        self.wp2.latitude = rospy.get_param('/waypoint/wp2')[0]
        self.wp2.longitude = rospy.get_param('/waypoint/wp2')[1]
        self.wp2.altitude = rospy.get_param('/waypoint/wp2')[2]

        self.wp3.latitude = rospy.get_param('/waypoint/wp3')[0]
        self.wp3.longitude = rospy.get_param('/waypoint/wp3')[1]
        self.wp3.altitude = rospy.get_param('/waypoint/wp3')[2]

        rospy.loginfo('Set Waypoint')

    def setObstaclePoints(self):
        self.obstacle.latitude = rospy.get_param('/obstacle')[0]
        self.obstacle.longitude = rospy.get_param('/obstacle')[1]
        self.obstacle.altitude = rospy.get_param('/obstacle')[2]

        rospy.loginfo('Set Obstacle')

    def stateCb(self, msg):
        prev_state = self.current_state

        self.current_state = msg
        
        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current mode: %s" % self.current_state.mode)

        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
        
    def homeCb(self, msg):
        self.home_position.latitude = msg.geo.latitude
        self.home_position.longitude = msg.geo.longitude
        self.home_position.altitude = msg.geo.altitude - self.diff

    def positionCb(self, msg):
        self.current_position.status = msg.status.status   # No_Fix = -1, Fix = 0, SBAS_Fix = 1, GBAS_Fix = 2

        self.current_position.latitude = msg.latitude
        self.current_position.longitude = msg.longitude
        self.current_position.altitude = msg.altitude - self.diff

    def pub_global_position(self, point):
        pose = GeoPoseStamped()

        pose.header.stamp = rospy.Time.now()

        pose.pose.position.latitude = point.latitude
        pose.pose.position.longitude = point.longitude
        pose.pose.position.altitude = point.altitude

        self.global_pose_pub.publish(pose)
    
    def pub_global_velocity(self, velocity):
        self.velocity_pub.publish(velocity)

    def waypoint_reach_check(self, wp, current_position):
        # Earth long radius = 6370km, short radius = 6262km
        # Get radius by linear interpolation

        diffLat = (wp[1].latitude - current_position.latitude) * pi/180  # unit(radian)
        diffLon = (wp[1].longitude - current_position.longitude) * pi/180  # unit(radian)
        diffAlt = wp[1].altitude - current_position.altitude

        radius = (6370-108/90*abs(wp[1].latitude)) * 1e3  # unit(m)

        currentLat_rad = current_position.latitude * pi/180

        d = sqrt((diffLat * radius)**2 + (diffLon * radius * cos(currentLat_rad))**2 + diffAlt**2)

        if d < self.wpReachRange :
            rospy.loginfo('Done')
            return True
        else:
            rospy.loginfo_throttle(1, '%s Process <Distance: %.1f>'%(wp[0], d))
            return False

    def calculate_velocity(self, target, obstacle, current_position):
        velocity = Twist()

        velocity.linear.x = 10
        velocity.linear.y = 0
        velocity.linear.z = 0

        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = 0

        return velocity

    def process(self):
        process = {0:['Takeoff', self.wpTakeoff],
                   1:['WP1', self.wp1],
                   2:['WP2', self.wp2],
                   3:['WP3', self.wp3],
                   4:['Return', self.wpTakeoff],
                   5:['Land', self.home_position]}.get(self.step, 'END')

        if (process[0] == 'Takeoff') or (process[0] == 'WP2') or (process[0] == 'WP3') or (process[0] == 'Land'):
            self.pub_global_position(process[1])
        
        elif (process[0] == 'WP1') or (process[0] == 'Return'):
            #self.pub_global_position(process[1])
            velocity = self.calculate_velocity(process[1], self.obstacle, self.current_position)
            self.pub_global_velocity(velocity)
        else:
            rospy.loginfo('Mission Complete')
            quit()

        result = self.waypoint_reach_check(process, self.current_position)

        if result is True:
            if process[0] == 'Land':
                while self.current_state.armed is True:
                    rospy.sleep(1)

            self.step += 1

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('main', anonymous=True)
    
    flight = mission()

    rate = rospy.Rate(20.0)

    try:
        flight.check_FCU_connection()
        rospy.sleep(1)

        #flight.setHome()
        #rospy.sleep(1)
        
        flight.setWayPoints()
        flight.setObstaclePoints()
        rospy.sleep(1)
        
        flight.setMode("OFFBOARD")
        rospy.sleep(1)

        flight.setArm()

        while not rospy.is_shutdown():
            flight.process()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass