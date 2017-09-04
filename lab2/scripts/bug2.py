#!/usr/bin/env python
#import all required libraries
import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Vector3
import random
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import Pose, PoseArray
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy
from visualization_msgs.msg import Marker

def bug2(ranges,pub_obj1):
	is_goal_reached = False
	while not rospy.is_shutdown():
		range_values = []
		range_values = data.ranges
		#stopflag = 0
		twist_obj1 = Twist()
		stop=1
		if len(data.ranges)>0:			
			for i in range(0,200):
				if range_values[i] < 1:
					stop = 0
					break
			if stop == 0:
				#stop the robot
				twist_obj1.linear.x=0
				twist_obj1.linear.y=0
				twist_obj1.angular.z=random.randrange(3,7)
				pub_obj.publish(twist_obj1)
			else:
				#move the robot
				twist_obj1.linear.x=2
				twist_obj1.linear.y=0
				twist_obj1.angular.z=0
				pub_obj.publish(twist_obj1)
		rate.sleep()
	return



def callback(data):
	bug2(data.ranges,pub_obj1)

if __name__=='__main__':
	try:
		rospy.init_node('robot_laser_listener',anonymous=True)
		pub_obj1 = rospy.Publisher("/points",Marker,queue_size=1)
		rospy.Subscriber('base_scan',LaserScan,callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
