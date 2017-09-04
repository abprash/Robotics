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

def test_ransac(ranges,pub_obj1):
	
	counter = 0
	inlier = []
	angle_increment = 0.00872664619237
	angle_min = 1.57079637051
	points = []
	for r in ranges:
		counter+=1
		if (r < 3):
			x_coordinate = r * math.cos(angle_min + (angle_increment * counter) ) 
			y_coordinate = r * math.sin(angle_min + (angle_increment * counter) )
			z_coordinate = 0
			points.append((x_coordinate,y_coordinate))
	print (points)
	if(len(points) < 10):
		return
	best_random_point = []
	best_inlier_length = 0
	for k in range(0,30):
		print "--------------------------> k ",k
		temp_points = []
		temp_points = points [:]
		inlier = []
		best_inlier = []
		while(True):
			#random selection of 2 points from list
			random_points = random.sample(points,2)
			print(random_points[0],random_points[1])
			print(random_points[0][0],random_points[1][0])
			if ((random_points[0][0]-random_points[1][0]<>0)):
				#and (random_points[0][0]-random_points[1][0] > 0.1)
				break
		#calculate the line joining the 2 points
		#ax + by + c = 0 is format of the line
		line_coeff = []
		line_coeff = construct_line(random_points)
		print "===>>>",line_coeff
		index = 0
		for i in points :
			random_point = random.choice(points)
			#print(random_point[0][1])
			distance = calc_distance(i, line_coeff)
			print distance
			if (distance < 0.5):
				inlier.append(random_point)
				best_random_point = random_point
				inner_index = 0
				for i in temp_points:
					if (random_point == i ):
						temp_points.pop(inner_index)
					inner_index+=1
			index += 1
		current_inlier_length = len(inlier)
		if(best_inlier_length < current_inlier_length):
			best_inlier_length = current_inlier_length
			best_inlier = inlier
		best_random_point1 = []
		best_random_point1 = []
		magnitudes = []
		max_value = 0
		min_value = 0
		for inlier_points in inlier:
			best_random_point2 = []
			magnitudes.append(inlier_points[0] + inlier_points[1])
			if(inlier_points[0]+inlier_points[1] > max_value):
				best_random_point2 = inlier_points
				max_value = math.fabs(inlier_points[0])+math.fabs(inlier_points[1])
			if(inlier_points[0]+inlier_points[1] < min_value):
				best_random_point1 = inlier_points
				min_value = inlier_points[0] + inlier_points[1]


	index2 = magnitudes.index(max(magnitudes))
	index1 = magnitudes.index(min(magnitudes))

	#construct a marker between the farthest points in the inlier list

	line_marker=Marker()
	line_marker.header.frame_id = "base_laser_link"
	line_marker.type = Marker.LINE_STRIP
	line_marker.color.g = 1
	line_marker.color.a = 1
	line_marker.type = Marker.ADD
	line_marker.lifetime = rospy.Duration(0)
	line_marker.scale.x = 0.05
	best_random_point1 = inlier[index1]
	best_random_point2 = inlier[index2]
	print best_random_point1
	print best_random_point2
	p1 = Point()
	p1.x = best_random_point1[0]
	p1.y = best_random_point1[1]
	p1.z = 0.5
	p2 = Point()
	p1.x = best_random_point2[0]
	p1.y = best_random_point2[1]
	p1.z = 0.5
	line_marker.points.append(p1)
	line_marker.points.append(p2)
	pub_obj1.publish(line_marker)
	return

def calc_distance(point1 , line_coeff):
	nr = (line_coeff[0] * point1[0]) + (line_coeff[1]*point1[1]) + line_coeff[2]
	nr = math.fabs(nr)
	dr = math.sqrt(line_coeff[0]*line_coeff[0] + line_coeff[1]*line_coeff[1])
	return (nr/dr) 


def construct_line(random_points):
	d = (random_points[1][1] - random_points[0][1])/(random_points[1][0] - random_points[0][0])
	a = d
	b = -1
	c = random_points[0][1] - (random_points[0][0] * d)
	line = [a,b,c]
	#print ("line -->",line)
	return line

def callback(data):
	test_ransac(data.ranges,pub_obj1)

if __name__=='__main__':
	try:
		rospy.init_node('robot_laser_listener',anonymous=True)
		pub_obj1 = rospy.Publisher("/points",Marker,queue_size=1)
		rospy.Subscriber('base_scan',LaserScan,callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
