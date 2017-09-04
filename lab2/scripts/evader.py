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

#create a listener 
#listens to the laser range finder values



def robot_listener():
	rospy.loginfo("In the listener")
	rospy.init_node('robot_laser_listener',anonymous=True)
	#keeping anonymous=true so that same named node won't be closed
	rospy.Subscriber('base_scan',LaserScan,callback)
	#callback will be executed once it starts subscribing
	rospy.spin()

def callback(data):
	#continue listening to the publisher
	#create publisher object with topic name, message type, and buffer size
	pub_obj = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	#initialize the publisher object with its name and anonymous type
	#rospy.init_node('robot_talker',anonymous=True)
	#set the frequency in which to publish
	rate = rospy.Rate(10)
	#create the twist object
	twist_obj = Twist()
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

#create the main block
if __name__=='__main__':
	try:
		robot_listener()
	except rospy.ROSInterruptException:
		pass
#general procedure
#create the publisher object
#start publishing the velocity value of 2 to the robot in the playground
#verify if the  robot moves
#get the type and value to be published to that topic
