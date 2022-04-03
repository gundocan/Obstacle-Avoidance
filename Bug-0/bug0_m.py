#!/usr/bin/env python

######### IMPORTS ###############

import rospy
import numpy as np
import ros_numpy
from numpy import transpose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
import time
import tf
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
import tf2_ros



class bug():
    def __init__(self):
        rospy.init_node('bug0', anonymous=True)
        self.scann = rospy.Subscriber("/scan", LaserScan, self.scan_callback)  # listen to Scan
        self.odo = rospy.Subscriber("/odom", Odometry, self.pose_callback)  # listen to odometry
        self.cmd_vel_topic = "/cmd_vel"
        self.rviz_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped,  self.destcallback)  # listen to Rviz 2D nav goal
        # self.odometry_frame = rospy.get_param("~odom_frame", "odom") #get odom frame
        # print("ODO: ", self.odometry_frame)
        # self.baseFootprint_frame = rospy.get_param("~base_footprint_frame", "base_footprint") #get base_footprint frame
        # print("base :", self.baseFootprint_frame)

        # self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # try:   #transformation from base_footprint to odom frame
        #   transform_goal = self.tf_buffer.lookup_transform(self.baseFootprint_frame, self.odometry_frame,
        #                                                    rospy.Time(), rospy.Duration(0.5))
        #  print("Transformation working")
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #  rospy.logwarn("Cannot get the goal position! ")
        # self.T = ros_numpy.numpify(transform_goal.transform)

        #x_goal = rviz_goal.pose.pose.position.x
        #y_goal = rviz_goal.pose.pose.position.y
        self.velocity_publisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)  # Velocity  publish
        self.pose_pub = rospy.Publisher("/global_goal", PoseStamped, queue_size=3)
        self.x_robot = 0
        self.y_robot = 0
        self.yaw = 0
        self.min_value_right = 0
        self.min_value_front = 0
        self.min_value_left = 0
        self.is_stop_moving = False
        self.vel_msg = Twist()
        rospy.wait_for_message("/move_base_simple/goal", PoseStamped)  # wait for 2D-nav goal from Rviz
        time.sleep(2.0)


        while True:
            # Print data.
            #print(">>>>> Data is below ")
            #print("min_value_right: " + str(self.min_value_right))
            #print("min_value_left: " + str(self.min_value_left))
            #print("min_value_front: " + str(self.min_value_front))
            print("yaw: " + str(self.yaw))
            if self.min_value_front > 0.4:
                # If robot is not following wall...
                if not ((self.min_value_right < 0.2 and self.yaw < 0) or (self.min_value_left < 0.2 and self.yaw > 0)):
                    self.rotate_to_goal_state()
                    self.is_stop_moving = False
                while self.min_value_front > 0.4:
                    self.move_forward_state()
                    # If robot is not following wall...
                    if not ((self.min_value_right < 0.2 and self.yaw < 0) or (self.min_value_left < 0.2 and self.yaw > 0)):
                        # If robot is not in the direction of the goal...
                        if (not self.is_towards_goal()) and not self.is_stop_moving:
                            print('..... I FUCKED UPPPPP ...')
                            #print(">>>>> Stop moving forward")
                            self.vel_msg.angular.z = 0.0
                            self.vel_msg.linear.x = 0.0
                            self.velocity_publisher.publish(self.vel_msg)
                            self.is_stop_moving = True
                            break

            else:
                self.follow_wall_state()
                print('ELSEEEEEEE')
            # Do some cleaning.
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = 0.0
            self.velocity_publisher.publish(self.vel_msg)

            #print(">>>>> Iteration Completed")

            # Check is goal reached.
            self.is_reached = self.is_goal_reached()
            if self.is_reached == True:
                print(">>>>> Goal is reached")
                #self.vel_msg.angular.z = 0.0
                #self.vel_msg.linear.x = 0.0
                #self.velocity_publisher.publish(self.vel_msg)
                print('Im done moving !!!!')
                break

    def scan_callback(self,scan_data):

        ranges = scan_data.ranges
        front_range = ranges[345:359] + ranges[0:15]
        right_range = ranges[85:95]
        left_range = ranges[265:275]
        self.min_value_left = min(left_range)
        self.min_value_right = min(right_range)
        self.min_value_front = min(front_range)

    def pose_callback(self, pose_data):
        self.x_robot = pose_data.pose.pose.position.x
        self.y_robot = pose_data.pose.pose.position.y

        quaternion = (
            pose_data.pose.pose.orientation.x,
            pose_data.pose.pose.orientation.y,
            pose_data.pose.pose.orientation.z,
            pose_data.pose.pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = rpy[2]

    def rotate_to_goal_state(self):  # Rotate towards goal.
        print(">>>>> rotate to goal state ")

        desired_angle_goal = math.atan2(abs(self.y_goal - self.y_robot), abs(self.x_goal - self.x_robot))
        K_angular = 0.5
        angular_speed = (desired_angle_goal - self.yaw) * K_angular
        while True:
            self.vel_msg.angular.z = angular_speed
            self.velocity_publisher.publish(self.vel_msg)
            if desired_angle_goal < 0:
                if ((desired_angle_goal) - self.yaw) > -0.1:
                    break
            else:
                if ((desired_angle_goal) - self.yaw) < 0.1:
                    break
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def move_forward_state(self):
        print(">>>>> Move forward")

        self.vel_msg.linear.x = 0.2
        self.velocity_publisher.publish(self.vel_msg)
        return

    def is_goal_reached(self):  # Check if goal is reached.
        if (self.x_goal - 0.09 < self.x_robot < self.x_goal + 0.09) and (self.y_goal - 0.09 < self.y_robot < self.y_goal + 0.09):
            return True
        return False


    def follow_wall_state(self):
        if self.min_value_right > self.min_value_left:
            angle = self.get_wall_angle(True)
            print(">>>>> Change angle to right (90 degrees): ", angle)
        else:
            angle = self.get_wall_angle(False)
            print(">>>>> Change angle to left (90 degrees): ", angle)
        if (angle - self.yaw) < 0:
            while (angle - self.yaw) < -0.1:
                self.vel_msg.angular.z = (angle - self.yaw) * 0.2   #0.9
                print('my angular speed is 1 :') # , self.vel_msg.angular)
                self.velocity_publisher.publish(self.vel_msg)
        else:
            while (angle - self.yaw) > 0.1:
                self.vel_msg.angular.z = (angle - self.yaw) * 0.2   #0.9
                print('my angular speed is 2 :') #, self.vel_msg.angular)
                self.velocity_publisher.publish(self.vel_msg)
        self.vel_msg.angular.z = 0
        # Move robot forward (for amount of time) after each rotation to prevent it from getting stuck.
        if self.min_value_front > 0.4:
            print('Sacma yerdesin')
            time = 0
            t0 = rospy.Time.now().to_sec()
            while time < 1.8 and self.min_value_front > 0.4:
                t1 = rospy.Time.now().to_sec()
                time = t1 - t0
                self.vel_msg.linear.x = 0.2
                self.velocity_publisher.publish(self.vel_msg)
                print('my angular speed is 3 :')  #, self.vel_msg.linear)


    def get_wall_angle(self, is_right):  # Get the wall angle.(in terms of turn left or right
        angle = 0
        if ((self.min_value_left > 0.6 and self.yaw > 0) or (self.min_value_right > 0.6 and self.yaw < 0)) and not (
                (0.0 < self.yaw < 0.002) or (-3.16 < self.yaw < -3.0)):
            angle = math.pi  if is_right else -math.pi  # math pi yerine 0 vardi
        else:
            angle = -math.pi / 2 if is_right else math.pi / 2
        return angle


    def destcallback(self, goal_data):

        # Get Rviz 2D-nav goal x and y coordinates
        self.x_goal = goal_data.pose.position.x
        self.y_goal = goal_data.pose.position.y
        self.pose_pub.publish(goal_data)

    def is_towards_goal(self): # Check if robot in direction of the goal.
        desired_angle_goal = math.atan2(abs(self.y_goal-self.y_robot), abs(self.x_goal-self.x_robot))
        if abs((desired_angle_goal)-self.yaw) < 0.2: #0.4 degistirdin
            return True
        return False



if __name__ == '__main__':

     b = bug()
     rospy.spin()


