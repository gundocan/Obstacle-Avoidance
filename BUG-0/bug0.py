#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from geometry_msgs.msg import Twist
import time
import tf
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
from std_srvs.srv import *
import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
desired_position_ = Point()
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    max_range_value = msg.range_max
    min_range_value = msg.range_min
    fr =msg.ranges[0:18] + msg.ranges[341:359]
    fr = [x for x in fr if min_range_value <= x <= max_range_value]
    lfr =msg.ranges[304:340]
    lfr = [x for x in lfr if min_range_value <= x <= max_range_value]
    rfr = msg.ranges[19:54]
    rfr = [x for x in rfr if min_range_value <= x <= max_range_value]
    lf = msg.ranges[270:303]
    lf = [x for x in lf if min_range_value <= x <= max_range_value]
    r = msg.ranges[54:89]
    r = [x for x in r if min_range_value <= x <= max_range_value]

    regions_ = {
        'right':  min(min(r), 10),
        'fright': min(min(rfr), 10),
        'front':  min(min(fr), 10),
        'fleft':  min(min(lfr), 10),
        'left':   min(min(lf), 10),
    }
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
        
def destcallback( goal_data):

   # Get Rviz 2D-nav goal x and y coordinates
    desired_position_.x  = goal_data.pose.position.x
    desired_position_.y = goal_data.pose.position.y


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    
    rospy.init_node('bug0')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    rviz_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, destcallback)  # listen to Rviz 2D nav goal
    rospy.wait_for_message("/move_base_simple/goal", PoseStamped)
    change_state(0)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        if state_ == 0:
            if regions_['front'] > 0.15 and regions_['front'] < 1:
                change_state(1)
        
        elif state_ == 1:
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            
            # less than 30 degrees
            if math.fabs(err_yaw) < (math.pi / 6) and \
               regions_['front'] > 1.5 and regions_['fright'] > 1 and regions_['fleft'] > 1:
                print 'less than 30'
                change_state(0)
            
            # between 30 and 90
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['left'] > 1.5 and regions_['fleft'] > 1:
                print 'between 30 and 90 - to the left'
                change_state(0)
                
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['right'] > 1.5 and regions_['fright'] > 1:
                print 'between 30 and 90 - to the right'
                change_state(0)
            
        rate.sleep()




if __name__ == "__main__":
    main()
