#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import geometry_msgs.msg
import turtlesim.srv
roslib.load_manifest('lab2')
import tf
import turtlesim.msg
from visualization_msgs.msg import Marker
import random
flagWall = 0
points = []
ranges = np.zeros((361, 2))
pos = 0
orien = 0
front_wall = False
left_line = False
on_line = False
def init():
    rospy.init_node('bug2', anonymous=True)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback)

def callback(data):
    global ranges
    global front_wall, left_line
    range = data.ranges
    range = np.array(range)
    counter = 0
    maxsize = 120

    for i in np.arange(maxsize):
        if range[180-(maxsize/2)+i] < 1:
            counter += 1
    # print range
    print "Printing Counter value for front_wall", counter
    if counter > 1:
        front_wall = True
    else:
        front_wall = False

    #resetting the counter value for checking left_line
    counter = 0
    maxsize = 80
    # print "printing range value for side linE", range[0:160]
    for i in np.arange(maxsize):
        if range[i] < 1:
            counter += 1
    print "Printing Counter value for left_line", counter
    if counter > 10:
        left_line = True
    else:
        left_line = False
            
# def isobstacleinwat(data):

def callbacktruth(data):
    global orien
    global pos
    pos = data.pose.pose.position
    orien = data.pose.pose.orientation
    # print msgorien.x, msgorien.y, msgorien.z, msgorien.w
    # print math.degrees(2 * np.arcsin(msgorien.z))
    # print "NEW MSG"

# 
def get_angluar_velocity(goal_angle):
    global front_wall, on_line
    if on_line:
        return min(goal_angle, 1)
    elif front_wall:
        return 1
    else:
        return min(goal_angle, 1)
    
def get_angluar_velocity_line_follow(goal_angle):
    global left_line, front_wall
    #if there is a left line then 
    # no need of angular velocity
    if front_wall:
        return 0.5
    if left_line:
        return 0
    else:
        return -1 * 0.4

def check_point_on_line(points):
    # Ax * (By - Cy) + Bx * (Cy - Ay) + Cx (Ay - By)
    global pos
    A = points[0,:]
    B = points[1,:]
    C = np.array([pos.x, pos.y])
    area = abs((A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0)
    threshold = 0.6
    print area
    print "ABOVE THE AREA VALUE"
    if (area < threshold):
        return True
    else:
        return False

def bug2():
    global front_wall, orien, pos, on_line
    pubcmd_vel = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size = 10)
    sub_truth = rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callbacktruth)
    rate = rospy.Rate(10)
    points = np.array([[-8, -2],[4.5, 9.0]])
    # Global Line equation from starting point and destination point
    distance = math.sqrt((points[1,1] - 0) ** 2 + (points[1,0] - 0) ** 2)
    atGoal = False
    thresholddist = 0.5
    follow_type = "GOAL_SEEK"
    while not atGoal:
        # print pos, orien
        if orien <> 0:
            robot_angle = 2 * np.arcsin(orien.z)
            disttemp = math.sqrt((points[1, 0] - pos.x) ** 2 + (points[1, 1] - pos.y) ** 2)
            goal_angle = math.atan((points[1, 1] - pos.y) / (points[1,0]- pos.x)) - robot_angle
            on_line = check_point_on_line(points)
            print "Printing on_line value",on_line
            print "Printing left_line value",left_line
            print "Printing front_wall value",front_wall
            print "Distance from dest", disttemp
            twist = Twist()
            print follow_type
            if disttemp < thresholddist:
                # reached at goal position
                #stop the robot
                twist.linear.x = 0 
                twist.angular.z = 0
                break
            else:
                vel = 0.0
                if front_wall:
                    vel = 0.0
                else:
                    vel = 0.6
                twist.linear.x = vel
                if follow_type == "GOAL_SEEK":
                    twist.angular.z =  get_angluar_velocity(goal_angle)
                    if front_wall:
                         follow_type = "LINE_FOLLOW"
                else:
                    twist.angular.z = -1 * get_angluar_velocity_line_follow(goal_angle)
                    if on_line and not front_wall:
                        follow_type = "GOAL_SEEK"
            pubcmd_vel.publish(twist)
            rate.sleep()    
          
if __name__ == '__main__':
    try:
        rospy.init_node('bug2', anonymous=True)
        # talker()
        init()
        bug2()
    except rospy.ROSInterruptException:
        pass

