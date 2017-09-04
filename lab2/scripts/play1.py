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

#create a listener 
#listens to the laser range finder values

value_x =0
value_y = 0


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
	pub_obj = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	#odom_data = rospy.Subscriber('odom',Odometry,callback2)
	pub_obj2 = rospy.Publisher('/mylines',LaserScan,queue_size=10)	
	#initialize the publisher object with its name and anonymous type
	#rospy.init_node('robot_talker',anonymous=True)
	#set the frequency in which to publish
	rate = rospy.Rate(10)
	#point_coordinates = Pose()
	point_coordinates_list = []
	#create the twist object
	twist_obj = Twist()
	odometry_obj = Odometry()
	laserscan_data = LaserScan()
	while not rospy.is_shutdown():
		range_values = []
		range_values = self.ranges
		#stopflag = 0
		twist_obj1 = Twist()
		stop=1	
		direction=0
		if len(self.ranges)>0:			
			for i in range(0,len(self.ranges)):
				laserscan_data = self
				value_x = math.cos(self.angle_min) * self.ranges[i]
				value_y = math.sin(self.angle_min) * self.ranges[i]
				#odometry_obj.pose.pose.position.x = value_x
				#odometry_obj.pose.pose.position.y = value_y
				
      				
				#add that to the point list
				#point_coordinates.append(x)
				#point_coordinates.append(y)
				#pub_obj2.publish(point_coordinates)
				#point_coordinates_list.append(point_coordinates)
				#rospy.loginfo(point_coordinates_list)
				#f = open ('samplefile.txt','w')
				#f.write(str(point_coordinates_list))
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
		return

def callback2(data):
	#pub_obj2 = rospy.Publisher('/mylines',Odometry,queue_size=10)
	#odometry_obj = Odometry()	
	#odometry_obj.pose.pose.position.x = value_x
	#odometry_obj.pose.pose.position.y = value_y
	#odometry_obj.header.stamp = rospy.Time.now()
      	#odometry_obj.header.frame_id = data.header.frame_id # i.e. '/odom'
      	#odometry_obj.child_frame_id = data.child_frame_id
	#pub_obj2.publish(odometry_obj)
	return 

def getFrame_id():
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
