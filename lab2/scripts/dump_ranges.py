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
	f = file("rangesdata.txt","w")
	f.write(data.ranges)
	f.close()

	#line_list_marker.points = point
	#publisher_obj3.publish(line_list_marker)
	
if __name__=='__main__':
	try:
		robot_listener()
	except rospy.ROSInterruptException:
		pass