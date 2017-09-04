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
from visualization_msgs.msg import *

def robot_listener():
	rospy.loginfo("In the listener")
	rospy.init_node('robot_laser_listener',anonymous=True)
	#keeping anonymous=true so that same named node won't be closed
	rospy.Subscriber('base_scan',LaserScan,callback)
	#callback will be executed once it starts subscribing
	rospy.spin()


def callback(data):
	#publisher object to make the robot move along the world
	pub_obj = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	#test publisher
	pub_obj2 = rospy.Publisher('/myvalues',numpy_msg(Floats),queue_size=1)
	#test publisher
	publisher_obj3 = rospy.Publisher('/points',PoseArray,queue_size = 10)	
	#extract the laser scan ranges values
	rate = rospy.Rate(10)
	ranges_data = []
	counter = 0
	x_coordinate = 0
	y_coordinate = 0
	z_coordinate = 0
	while not rospy.is_shutdown():
		ranges_data = data.ranges
		pose_array = PoseArray()
		markerid = 0
		for i in data.ranges:
			for i in data.ranges:
			#marker = Marker()
				data.ranges = data.ranges
				if(counter == len(ranges_data) - 1):
					counter = 0
					markerid = 0
					#pose_array.poses.pop(0)
				if(i < 3 and i > 0):
					for i in data.ranges:	
						x_coordinate = i * math.cos(data.angle_min + (data.angle_increment * counter)) 
						y_coordinate = i * math.sin(data.angle_min + (data.angle_increment * counter))
						z_coordinate = 1
						pose = Pose()
       					pose.position.x = x_coordinate
       					pose.position.y = y_coordinate
       					pose.position.z =  z_coordinate
       					pose_array.poses.append(pose)
       					counter+=1
       			pose_array.header.frame_id = "base_link"
       			publisher_obj3.publish(pose_array)
       		#pose_array = PoseArray()
		#rospy.spin()
		#rate.sleep()
	return

if __name__=='__main__':
	try:
		robot_listener()
	except rospy.ROSInterruptException:
		pass