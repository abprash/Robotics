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
	publisher_obj3 = rospy.Publisher('/points',Marker,queue_size = 10)	
	rospy.Subscriber('base_scan',LaserScan,callback,publisher_obj3)
	#callback will be executed once it starts subscribing
	rospy.spin()


def callback(data,publisher_obj3):
	line_list_marker = Marker()
	line_list_marker.id=0
	line_list_marker.header.frame_id = "/base_link"
	line_list_marker.color.g=1
	line_list_marker.color.a=1
	line_list_marker.action = Marker.ADD
	line_list_marker.type = Marker.LINE_STRIP
	ranges_data = []
	intensities = []
	counter = 0
	for (i,i1) in zip(data.ranges,data.intensities):
		counter+=1
		flag = 0
		if (i < 3 and i1 == 1):
			x_coordinate = i * math.cos(data.angle_min + (data.angle_increment * counter)) 
			y_coordinate = i * math.sin(data.angle_min + (data.angle_increment * counter))
			z_coordinate = 0
			point = Point()
			point.x = x_coordinate
			point.y = y_coordinate
			point.z = 1
			points = []
			points.append(point)
			break
	ranges_data = data.ranges
	intensities = data.intensities
	#ranges_data = ranges_data.reverse()
	#intensities = intensities.reverse()
	for (i,i1) in zip(ranges_data,intensities):
		counter+=1
		flag = 0
		if (i < 3 and i1 == 1):
			x_coordinate = i * math.cos(data.angle_min + (data.angle_increment * counter)) 
			y_coordinate = i * math.sin(data.angle_min + (data.angle_increment * counter))
			z_coordinate = 0
			point = Point()
			point.x = x_coordinate
			point.y = y_coordinate
			point.z = 1
			points = []
			points.append(point)
			break
	line_list_marker.points = point
	publisher_obj3.publish(line_list_marker)
	
if __name__=='__main__':
	try:
		robot_listener()
	except rospy.ROSInterruptException:
		pass