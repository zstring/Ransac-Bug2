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
sinx = np.array((361,1)) 
cosx = np.array((361,1))
points = []
pointX = []
pointY = []
ranges = np.zeros((361, 2))
def init():
    global sinx
    global cosx
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback)

    degrees = np.linspace(np.pi/2,-1 * np.pi / 2 , 361)
    sinx = np.sin(degrees)
    cosx = np.cos(degrees)


def callback(data):
    global ranges
    global sinx
    global cosx
    global front_wall, left_line
    range = data.ranges
    range = np.array(range)
    range[range==3.0] = 0
    y = range * sinx
    x = range * cosx
    ranges[:,0:1] = x.reshape((x.shape[0], 1))
    ranges[:,1:] = y.reshape((y.shape[0], 1))
    
    counter = 0
    maxsize = 40
    for i in np.arange(maxsize):
        if range[180-maxsize+i] < 1:
            counter += 1
    print counter
    print "COUNTER CHECKING"
    if counter > 10:
        front_wall = True
    else:
        front_wall = False

    for i in np.arange(maxsize):
        if range[360-i] < 1:
            counter += 1
    if counter > 10:
        left_line = True
    else:
        left_line = False

#Ransac Implementation
def ransacimp1(x, y, iter, threshold, point_threshold):

    x = ranges[:,0]
    y = ranges[:,1]
    ln = len(x)
    indexes = range(ln)
    global pointX, pointY
    points = []
    pointX = [-1]
    pointY = [-1]
    somethreshold = ln * 0.02
    k = iter
    
    mx = -1
    ymx = -1
    mn = -1
    ymn = -1
    print "ENTRY in the ransacimp function "
    for r in range(5):
#         threshold = threshold + 0.2
        inl = []
        outl = []
        #print indexes

        print "PRIINTING INDEX"
        for i in range(k):
            tempinl = []
            tempoutl = []
            
            index1 = random.randint(0,len(indexes)-1)
            index2 = random.randint(0,len(indexes)-1)
            if index1 == index2:
                continue
            x1 = x[indexes[index1]]
            y1 = y[indexes[index1]]
            
            x2 = x[indexes[index2]]
            y2 = y[indexes[index2]]
            if x1 == 0 or x2 == 0:
               continue
            for j in (indexes):
                x0 = x[j]
                y0 = y[j]
                if x0 == 0 and y0 == 0:
		    continue
                dist = abs((y2 - y1) * x0 - (x2 - x1)*y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1))
                if dist < threshold:
                    # add inliners
                    tempinl.append(j)
                else:
                    # add outliers
                    tempoutl.append(j)
            if len(inl) < len(tempinl):
                inl = tempinl
                outl = tempoutl
                mx, ymx, mn, ymn = getmaxindex(inl, indexes, index1, index2)
                in1 = index1
                in2 = index2
            
        #Add new line two something for latter use
        if len(inl) > point_threshold:
            print "Inliers Count"
            print len(inl)
            pointX.append(mx)
            print "after ading"
            pointY.append(ymx)
            pointX.append(mn)
            pointY.append(ymn)
            indexes = outl
        if len(indexes) < somethreshold:
            print r
            print "R VALUE"
	    break


def getmaxindex(inl, indexes, index1, index2):
    mx = np.max(ranges[inl,0]) 
    mn = np.min(ranges[inl,0])
    
    p1 = ranges[indexes[index1],:]
    p2 = ranges[indexes[index2],:]
    yout = ranges[inl,:]
    ymx = yout[np.argmax(yout[:,0]), 1]
    ymn = yout[np.argmin(yout[:,0]), 1]
    return mx, ymx, mn, ymn
            
# def isobstacleinwat(data):
pos = 0
orien = 0
front_wall = False
left_line = False
on_line = False
def callbacktruth(data):
    global orien
    global pos
    pos = data.pose.pose.position
    orien = data.pose.pose.orientation
    # print msgorien.x, msgorien.y, msgorien.z, msgorien.w
    # print math.degrees(2 * np.arcsin(msgorien.z))
    print "NEW MSG"

def get_angluar_velocity(goal_angle):
    global front_wall, on_line
    if front_wall:
        return 0
    elif on_line:
        return 0
    else:
        return min(goal_angle, 0.1)
    
def get_angluar_velocity_line_follow(goal_angle):
    global left_line
    if left_line:
        return 1
    else:
        return 1

def check_point_on_line(points):
    # Ax * (By - Cy) + Bx * (Cy - Ay) + Cx (Ay - By)
    global pos
    A = points[0,:]
    B = points[1,:]
    C = np.array([pos.x, pos.y])
    area = (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0
    threshold = 0.5
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
    points = np.array([[0, 0],[8, 8]])
    # Global Line equation from starting point and destination point
    distance = math.sqrt((8 - 0) ** 2 + (8 - 0) ** 2)
    atGoal = False
    thresholddist = 0.5
    follow_type = "GOAL_SEEK"
    while not atGoal:
        print pos, orien
        if orien <> 0:
            robot_angle = 2 * np.arcsin(orien.z)
            disttemp = math.sqrt((8 - pos.x) ** 2 + (8 - pos.y) ** 2)
            goal_angle = math.atan((8 - pos.y) / (8 - pos.x)) - robot_angle
            on_line = check_point_on_line(points)
            print on_line
            print "PRINITNT ON LINE VALUE"
            twist = Twist()
            print follow_type
            if (disttemp < thresholddist):
                # reached at goal position
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
            else:
                vel = 0.0
                if front_wall:
                    vel = 0.0
                else:
                    vel = 1
                twist.linear.x = vel
                if follow_type == "GOAL_SEEK":
                    twist.angular.z = get_angluar_velocity(goal_angle)
                    if front_wall:
                        follow_type = "LINE_FOLLOW"
                else:
                    twist.angular.z = get_angluar_velocity_line_follow(goal_angle)
                    if on_line:
                        follow_type = "GOAL_SEEK"
            pubcmd_vel.publish(twist)
            rate.sleep()    
        
def talker():
    rate = rospy.Rate(10)
    shape = Marker.LINE_STRIP
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    pub1 = rospy.Publisher('vis_actual', Marker, queue_size=10)
    # initialization
    init()
    global ranges
    iter = 200
    threshold = 0.1
    point_threshold = 3
    x = 1
    y = 1
    marker = Marker()
    while not rospy.is_shutdown():
        try:
            bug2()
            marker.header.frame_id = "/my_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "bug2"
            marker.id = 0
            marker.type = shape
            marker.action = Marker.ADD

            ln = len(pointX)
            for i in range(1,ln):
                p = Point()
                #print ranges[i,0]
                p.x = pointX[i]
                p.y = pointY[i]
                marker.points.append(p)

	    marker.scale.x = 0.1
	    marker.scale.y = 0.0
	    marker.scale.z = 0.0 
            
	    marker.color.r = 1.0
	    marker.color.g = 0.0
	    marker.color.b = 0.0
	    marker.color.a = 1.0

	    marker.lifetime = rospy.Duration()
            marker1.lifetime = rospy.Duration()
            #publishing the marker 
	    pub.publish(marker)
            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue		
            



if __name__ == '__main__':
    try:
        rospy.init_node('bug2', anonymous=True)
        # talker()
        init()
        bug2()
    except rospy.ROSInterruptException:
        pass

