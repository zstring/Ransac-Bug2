#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
roslib.load_manifest('lab1')
import tf
import turtlesim.msg

flagWall = 0


def callback(data):
    range = data.ranges	
    counter = 0
    global flagWall
    maxSize = 50
    for i in xrange(maxSize):
        if range[180-maxSize+i] < 1:
            counter += 1
    if counter > 10:
       flagWall = 1
    
def listener():
    rospy.init_node('evader', anonymous=True)
    rospy.Subscriber("/robot_0/base_scan", LaserScan, callback)
    #rospy.spin()

def talker():
    listener()
    pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    rospy.init_node('evader', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    global flagWall 
    while not rospy.is_shutdown():
	twist = Twist()
        if flagWall == 1:
		flagWall = 0
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
        	twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 1.3
	else:
        	twist.linear.x = 2
	        twist.linear.y = 0
        	twist.linear.z = 0
	        twist.angular.x = 0
		twist.angular.y = 0
	        twist.angular.z = 0
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
	rospy.init_node('evader', anonymous=True)
    	talker()
    except rospy.ROSInterruptException:
	print ("INSIDE EXCEPT")
        pass
