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

def robot_listener():
	rospy.loginfo("In the listener")
	rospy.init_node('robot_laser_listener',anonymous=True)
	#keeping anonymous=true so that same named node won't be closed
	rospy.Subscriber('base_scan',LaserScan,callback)
	#callback will be executed once it starts subscribing
	rospy.spin()


def callback(data):
	#publisher object to make the robot move along the world
	#test publisher
	#test publisher
	publisher_obj3 = rospy.Publisher('/points',Marker,queue_size = 10)	
	#extract the laser scan ranges values
	while not rospy.is_shutdown():
		line_list_marker = Marker()
		line_list_marker.header.frame_id = "base_link"
		line_list_marker.color.g=1
		line_list_marker.color.a=1
		line_list_marker.action = Marker.ADD
		#if counter2 == 361:
		#	counter2 = 0
		line_list_marker.id = 0
		line_list_marker.type = Marker.LINE_STRIP
		line_list_marker.scale.x = 0.1
		line_list_marker.points = [Point(random.randint(0,3),random.randint(0,3),0),Point(random.randint(0,3),random.randint(0,3),0)]
		#line_list_marker.points.append(y_coordinate)
		"""
		pose = Pose()
		pose.position.x = x_coordinate
		pose.position.y = y_coordinate
		pose.position.z =  1
		line_list_marker.pose = pose
		"""
		#counter+=1
		#pose_array.header.frame_id = "base_link"
		#line_list_marker.points.append(point)
		publisher_obj3.publish(line_list_marker)
   		#pose_array = PoseArray()
	#rospy.spin()
	#rate.sleep()
	return

if __name__=='__main__':
	try:
		robot_listener()
	except rospy.ROSInterruptException:
		pass