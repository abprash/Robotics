#!/usr/bin/python
import rospy
import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion
from math import sqrt
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


bag = rosbag.Bag('grid.bag','r')
pt_count = 0
pt_count1 = 0
#degree to radian conversion
deg2rads = 0.0174532925
variance = 10.0
angle_error = 5.0 * deg2rads
tag_positions = np.ndarray((6,),dtype=np.object)
grid = np.zeros((35,35,36))
threshold = 0.08
#(12,28,20)
grid[11][27][20]=1.0
 

class Grid_centre_class:
	x = 0.0
	y = 0.0
	z = 0.0
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z

grid_centres = np.ndarray(grid.shape,dtype=np.object)

def init_grid():
	#set the tag positions
	tag_positions[0] = Grid_centre_class(125,525,0) 
	tag_positions[1] = Grid_centre_class(125,325,0) 
	tag_positions[2] = Grid_centre_class(125,125,0) 
	tag_positions[3] = Grid_centre_class(425,125,0) 
	tag_positions[4] = Grid_centre_class(425,325,0) 
	tag_positions[5] = Grid_centre_class(425,525,0) 
	global grid_centres
	for i in range(grid.shape[0]):
		for j in range(grid.shape[1]):
			for k in range(grid.shape[2]):
				grid_centres[i][j][k] = Grid_centre_class(i*20+10,j*20+10,k*10*deg2rads)

#to,from
def get_rotation(line_angle,cell_position_angle):
	if(cell_position_angle - line_angle > 0 and cell_position_angle - line_angle < math.pi ):
		 return -1*(math.pi - abs(abs(line_angle - cell_position_angle) - math.pi)) 
	else:
		 return (math.pi - abs(abs(line_angle - cell_position_angle) - math.pi))

def get_min_rotation(line_angle, cell_position_angle, act):
	diff = line_angle - cell_position_angle
	if diff<0:
		diff = diff + (2*math.pi)
	else:
		diff = (2*math.pi) -diff
	return diff 
	

def calc_motion():
	
	global grid, grid_centres, threshold, pt_count
	#list comprehension to read the bag values
	bag_msgs = list(bag.read_messages(topics=['Movements','Observations']))
	bag_msgs.reverse()
	old_point = Point()
	old_point.x = grid_centres[11][27][20].x/70
	old_point.y = grid_centres[11][27][20].y/70
	msg_count = 0
	#they alternate
	while msg_count<89:
		topic, msg, time_stamp = bag_msgs.pop()
		if(msg==None):
			break
		if topic == 'Movements':
			#converting the quaternion data to euler angles
			rotation1 = euler_from_quaternion((msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w))[2]
			translation = msg.translation*100.0 
			#converting to cm
			rotation2 =  euler_from_quaternion((msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w))[2]
		topic = None		
		msg = None
		time_stamp = None
		try:
			msg_count +=1
			topic, msg, time_stamp = bag_msgs.pop()
		except IndexError:
			return		
		if topic == 'Observations':
			tagNum = msg.tagNum 
			range_val =	msg.range*100 
			bearing = euler_from_quaternion((msg.bearing.x,msg.bearing.y,msg.bearing.z,msg.bearing.w))[2]
		#creating a new grid to store the updated probability values
		new_probs = np.zeros(grid.shape,dtype=np.float64)
		for outer_i in range(grid.shape[0]):
			for outer_j in range(grid.shape[1]):
				for outer_k in range(grid.shape[2]):
					if(grid[outer_i][outer_j][outer_k]<threshold):
						continue
					src_centre = grid_centres[outer_i][outer_j][outer_k]
					for inner_i in range(grid.shape[0]):
						for inner_j in range(grid.shape[1]):
							for inner_k in range(grid.shape[2]):
								if outer_i==inner_i and outer_j==inner_j and outer_k==inner_k :
									continue
								dest_centre = grid_centres[inner_i][inner_j][inner_k]
								#Calculate trans between each point with every other point in the grid
								calc_dist = math.sqrt((dest_centre.y - src_centre.y)**2 + (dest_centre.x - src_centre.x)**2)
								#calculate the angle between the 2 chosen points with respect to x axis
								theta = math.atan2((dest_centre.y - src_centre.y),(dest_centre.x - src_centre.x))
								if theta<0:
									theta = 2*math.pi - abs(theta)
								#Calcuate r1	
								calc_rot1 = get_rotation(theta, src_centre.z)
								
								#Calculate r2
								calc_rot2 = get_rotation(dest_centre.z, theta)
								#difference between calculated and actual values
								error_r1 = calc_rot1 - rotation1
								error_trans = calc_dist - translation
								error_r2 = calc_rot2 - rotation2
								#calculating the probability density function
								prob_r1 = (1.0/(math.sqrt(2*math.pi*(angle_error))))*math.exp(-(error_r1**2)/(2*angle_error))
								prob_t = (1.0/(math.sqrt(2*math.pi)*sqrt(variance)))*math.exp(-(error_trans**2)/(2*variance))	
								prob_r2 = (1.0/(math.sqrt(2*math.pi*(angle_error))))*math.exp(-(error_r2**2)/(2*angle_error)) 	
								#get the new probability value in the current cell
								new_probs[inner_i][inner_j][inner_k] +=  grid[outer_i][outer_j][outer_k]*(prob_r1 * prob_t * prob_r2)
		grid = new_probs
		#get the index value of the cell with max probability
		max_prob = np.unravel_index(np.argmax(grid), grid.shape)
		prob_obs = np.zeros((grid.shape))
		for outer_i in range(grid.shape[0]):
			for outer_j in range(grid.shape[1]):
                        	for outer_k in range(grid.shape[2]):
					src_pos = grid_centres[outer_i][outer_j][outer_k]
					tag_pos = tag_positions[tagNum]
					calc_range = math.sqrt((tag_pos.y - src_pos.y)**2 + (tag_pos.x - src_pos.x)**2)
					#calculate bearing
					theta = math.atan2((tag_pos.y-src_pos.y),(tag_pos.x-src_pos.x))
					if theta<0:
						theta = 2*math.pi - abs(theta)
					calc_bearing = get_rotation(theta, src_pos.z)
					error_range = calc_range - range_val
					error_bearing = calc_bearing - bearing
					prob_range = (1.0/(math.sqrt(2*math.pi)*sqrt(variance)))*math.exp(-(error_range**2)/(2*variance))
					prob_bearing = (1.0/(math.sqrt(2*math.pi*(angle_error))))*math.exp(-(error_bearing**2)/(2*angle_error))
					prob_obs[outer_i][outer_j][outer_k] = prob_range * prob_bearing 
					grid[outer_i][outer_j][outer_k] = grid[outer_i][outer_j][outer_k]*(prob_range * prob_bearing)
		normalization = np.sum(grid)
		grid = grid/normalization
		max_prob = np.unravel_index(np.argmax(grid), grid.shape)

		global pt_count1
		#plot the tags
		tag_pos_marker = Marker()
		tag_pos_marker.header.frame_id = "/map"
		tag_pos_marker.header.stamp = rospy.Time.now()
		tag_pos_marker.action = Marker.ADD
		tag_pos_marker.color.b = 1.0;
		tag_pos_marker.ns = "tags"    	
		tag_pos_marker.type = Marker.POINTS
		tag_pos_marker.id = pt_count1		
		pt_count1 +=1
		for tag in range(6):
			tag_pos_point = Point()
			
	       		tag_pos_point.x = tag_positions[tag].x/70
		        tag_pos_point.y = tag_positions[tag].y/70
		       	tag_pos_point.z = 0
		       	tag_pos_marker.points.append(tag_pos_point)

		pub.publish(tag_pos_marker)
		#plot the robot's path using line strips 
		line_strip = Marker()
		line_strip.header.frame_id = "/map"
		line_strip.header.stamp = rospy.Time.now()
		line_strip.action = Marker.ADD
		line_strip.lifetime = rospy.Time(0)
		line_strip.scale.x = 0.1;
		line_strip.scale.y = 0.1;
		line_strip.scale.z = 0.1;
		line_strip.color.a = 1.0; 
		line_strip.color.r = 1.0;
		line_strip.ns = "pts_line"
		pt_count = pt_count + 1
		line_strip.id = (pt_count)
		line_strip.type = Marker.LINE_STRIP
		p1 = Point()
		p1.x = grid_centres[max_prob[0]][max_prob[1]][max_prob[2]].x/70
		p1.y = grid_centres[max_prob[0]][max_prob[1]][max_prob[2]].y/70
		line_strip.points.append(p1)
		line_strip.points.append(old_point)
		pub.publish(line_strip)
		rospy.sleep(rospy.Duration(10))
		old_point = p1

			
if __name__ == '__main__':
	rospy.init_node("grid_localization")
	#init grid
	init_grid()	 
	pub = rospy.Publisher("/visualization_topic", Marker, queue_size=10)	
	calc_motion()
	rospy.spin()