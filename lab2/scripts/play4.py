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
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import *
#create a listener 
#listens to the laser range finder values



def robot_listener():
	rospy.loginfo("In the listener")
	rospy.init_node('robot_laser_listener',anonymous=True)
	#keeping anonymous=true so that same named node won't be closed
	rospy.Subscriber('base_scan',LaserScan,callback)
	#callback will be executed once it starts subscribing
	rospy.spin()


def callback(self):
	#continue listening to the publisher
	#create publisher object with topic name, message type, and buffer size
	pub_obj = rospy.Publisher('cmd_vel',Twist,queue_size=1)
	pub_obj2 = rospy.Publisher('/myvalues',numpy_msg(Floats),queue_size=1)
	publisher_obj3 = rospy.Publisher('/lines', Marker,queue_size = 1)	
	#initialize the publisher object with its name and anonymous type
	#rospy.init_node('robot_talker',anonymous=True)
	#set the frequency in which to publish
	rate = rospy.Rate(10)
	#point_coordinates = Pose()
	point_coordinates_list = []
	point_coordinates = []
	#create the twist object
	twist_obj = Twist()
	odometry_obj = Odometry()
	laserscan_data = LaserScan()
	while not rospy.is_shutdown():
		value_x =0
		value_y = 0
		outlier = []
		inlier = []
		range_values = []
		range_values = self.ranges
		#stopflag = 0
		twist_obj1 = Twist()
		stop=1	
		direction=0
		if len(self.ranges)>0:			
			for i in range(0,359):
				laserscan_data = self
				value_x = self.ranges[i] * math.cos(self.angle_min+self.angle_increment)
				value_y = self.ranges[i] * math.sin(self.angle_min+self.angle_increment)
				point_coordinates.append(value_x)
				point_coordinates.append(value_y)
			coordinates_numpy = numpy.array(point_coordinates, dtype=numpy.float32)
		pub_obj2.publish(coordinates_numpy)
		"""
		while(True):
			random_point = random.randint(0,len(point_coordinates)/2)
			if(random_point % 2 == 0):
				#it is an x coordinate
				x_coordinate1 = point_coordinates[random_point]
				y_coordinate1 = point_coordinates[random_point+1]
			else:
				#it is a y coordinate
				y_coordinate1 = point_coordinates[random_point]
				x_coordinate1 = point_coordinates[random_point-1]

			random_point2 = random.randint(0,len(point_coordinates)/2)
			if(random_point2 % 2 == 0):
				#it is an x coordinate
				x_coordinate2 = point_coordinates[random_point]
				y_coordinate2 = point_coordinates[random_point+1]
			else:
				#it is a y coordinate
				y_coordinate2 = point_coordinates[random_point]
				x_coordinate2 = point_coordinates[random_point-1]
			if (x_coordinate2 - x_coordinate1)<>0:
				break
		c1 = (y_coordinate2 - y_coordinate1)/(x_coordinate2 - x_coordinate1) 
		a1 = c1
		b = -1
		counter = 0
		i1 = 0 
		while (counter < len(point_coordinates)):
			i1=counter 
			numerator = (c1*point_coordinates[counter]) + (-1 * point_coordinates[counter + 1]) + (y_coordinate1 - c1 * x_coordinate1)
			#numerator_sq = numerator * numerator
			numerator = abs(numerator)
			denom = math.sqrt((c1*c1) + 1 )
			distance = numerator/denom
 			if( distance < 1):
 				#adding the selected point to the list of inliers
 				inlier.append(point_coordinates[counter])
 				inlier.append(point_coordinates[counter + 1])
 			counter += 2
		#o = numpy.array(outlier, dtype=numpy.float32)
		i = numpy.array(inlier, dtype=numpy.float32)
		#pub_obj2.publish(i)
		counter2=0
		line_marker_array = MarkerArray()
		"""
		count = 180
		marker = Marker()
   		marker.header.frame_id = "/base_link"
   		marker.type = marker.SPHERE
   		marker.action = marker.ADD
   		marker.scale.x = 0.2
   		marker.scale.y = 0.2
   		marker.scale.z = 0.2
   		marker.color.a = 1.0
   		marker.color.r = 1.0
   		marker.color.g = 1.0
   		marker.color.b = 0.0
   		marker.pose.orientation.w = 1.0
   		marker.pose.position.x = math.cos(count / 50.0)
   		marker.pose.position.y = math.cos(count / 40.0) 
   		marker.pose.position.z = math.cos(count / 30.0)
    	publisher_obj3.publish(marker)
"""
				if range_values[i] < 1:
					stop = 0
					break
			if stop == 0:
				#stop the robot
				twist_obj1.linear.x=0
				twist_obj1.linear.y=0
				direction+=1
				while(True):
					if(direction%2==0):
						temp1 = random.randrange(3,7)
						if(temp1!=0):
							break
					else:
						temp1 = random.randint(-7,0)
						if(temp1!=0):
							break
				twist_obj1.angular.z=temp1
				pub_obj.publish(twist_obj1)
				#pub_obj2.publish(odometry_obj)
			else:
				#move the robot
				twist_obj1.linear.x=2
				twist_obj1.linear.y=0
				twist_obj1.angular.z=0
				pub_obj.publish(twist_obj1)
		pub_obj2.publish(laserscan_data)
		rate.sleep()
		#f.close()
"""


#create the main block
if __name__=='__main__':
	try:
		robot_listener()
	except rospy.ROSInterruptException:
		pass
