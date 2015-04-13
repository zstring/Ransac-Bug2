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
    range = data.ranges
    range = np.array(range)
    range[range==3.0] = 0
    y = range * sinx
    x = range * cosx
    ranges[:,0:1] = x.reshape((x.shape[0], 1))
    ranges[:,1:] = y.reshape((y.shape[0], 1))
    print "HOW ARE YOR"

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


def ransacimp():
    x = ranges[:,0]
    y = ranges[:,1]
    ln = len(x)
    indexes = range(ln)
    global points
    points = []
    somethreshold = ln * 0.15
    k = 300
    inl = []
    outl = []
    xx1 = -1
    yy1 = -1
    xx2 = -1
    yy2 = -1
    threshold = -0.1
    print "ENTRY in the ransacimp function "
    for r in range(4):
	    threshold = threshold + 0.2
	    for i in range(k):
		tempinl = []
		tempoutl = []
		index1 = random.randint(0,len(indexes)-1)
		index2 = random.randint(0,len(indexes)-1)
		x1 = x[indexes[index1]]
		y1 = y[indexes[index1]]
		x2 = x[indexes[index2]]
		y2 = y[indexes[index2]]
		for j in range(len(indexes)):
		    x0 = x[indexes[j]]
		    y0 = y[indexes[j]]
		    dist = abs((y2 - y1) * x0 - (x2 - x1)*y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1))
                    print dist
		    if dist < threshold:
		        # add inliners
		        tempinl.append(i)
		    else:
		        # add outliers
		        tempoutl.append(i)
		if len(inl) < len(tempinl):
		    inl = tempinl
		    outl = tempoutl
		    in1 = index1
		    in2 = index2
	    #Add new line two something for latter use
	    points.append(index1)
            points.append(index2)
	    indexes = outl
	    if len(indexes) < somethreshold:
		break

def getmaxindex(inl, indexes, index1, index2):
    mx = np.max(ranges[inl,0]) 
    mn = np.min(ranges[inl,0])
    
    p1 = ranges[indexes[index1],:]
    p2 = ranges[indexes[index2],:]
#    if p2[0] == p1[0]:
#        return mx, np.max(ranges[inl, 1]), mn, np.min(ranges[inl, 1])
#    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
#    ymx = m * (mx - p1[0]) + p1[1]
#    ymn = m * (mn - p1[0]) + p1[1]
#     print mx, ymx, mn, ymn
    yout = ranges[inl,:]
    ymx = yout[np.argmax(yout[:,0]), 1]
    ymn = yout[np.argmin(yout[:,0]), 1]
    return mx, ymx, mn, ymn
        
    

def talker():
    rate = rospy.Rate(10)
    shape = Marker.LINE_LIST
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    pub1 = rospy.Publisher('vis_actual', Marker, queue_size=10)
    # initialization of 
    init()
    global ranges
    iter = 200
    threshold = 0.1
    point_threshold = 3
    x = 1
    y = 1
    while not rospy.is_shutdown():
        try:
            ransacimp1(x, y, iter, threshold, point_threshold)
	    marker = Marker()
            marker1 = Marker()
            marker.header.frame_id = "/my_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "ransac"
            marker.id = 0
            marker.type = shape
            marker.action = Marker.ADD
            marker1.header.frame_id = "/my_frame"
            marker1.header.stamp = rospy.Time.now()
            marker1.ns = "ransac"
            marker1.id = 1
            marker1.type = Marker.LINE_STRIP
            marker1.action = Marker.ADD

            ln = len(pointX)
            for i in range(1,ln):
                p = Point()
                #print ranges[i,0]
                p.x = pointX[i]
                p.y = pointY[i]
                marker.points.append(p)

            for i in range(361):
                if ranges[i,0] == 0 and ranges[i,1] == 0:
			continue
		p = Point()
		p.x = ranges[i,0]
		p.y = ranges[i,1]
		marker1.points.append(p)
            '''marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            ''' 
	    marker.scale.x = 0.1
	    marker.scale.y = 0.0
	    marker.scale.z = 0.0 
            
	    marker.color.r = 1.0
	    marker.color.g = 0.0
	    marker.color.b = 0.0
	    marker.color.a = 1.0
 
            marker1.scale.x = 0.1
            marker1.scale.y = 0.0
            marker1.scale.z = 0.0

            marker1.color.r = 0.0
            marker1.color.g = 1.0
            marker1.color.b = 1.0
            marker1.color.a = 1.0

	    marker.lifetime = rospy.Duration()
            marker1.lifetime = rospy.Duration()
	    pub.publish(marker)
            pub1.publish(marker1)
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue		
            



if __name__ == '__main__':
    try:
        rospy.init_node('ransac', anonymous=True)
        #turtlename = rospy.get_param('~/robot_1/odom')
        talker()
    except rospy.ROSInterruptException:
        pass

