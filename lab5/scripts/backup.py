#!/usr/bin/python
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from math import sqrt
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Vector3
from nav_msgs.msg import Odometry

map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

#print map
final_path = []
path_counter = 0

def do_astar(data):
       """
       A star uses the f measure which is g and h
       g is the cost of travelling to each node
       h is the heuristic - which the system learns
       

       g is constant for all the neighbouring nodes except when there are any obstacles
       h is the euclidean distance which is calculated between itself and the destination,
       everytime for each and every new node
       the node with the least f value gets added to the path
       """

       #using the param service to get the end goal coordinates
       #goalx = rospy.get_param(goalx)
       #goaly = rospy.get_param(goaly)
       #destination = convert_coord_to_map(goalx,goaly)
       source = convert_coord_to_map(-8,-2)
       destination = convert_coord_to_map(4.5,9)
       final_path = path_find(source, destination)
       robot_move(final_path)

def callback(data):
       global path_counter
       global final_path
       twist_obj = Twist()
       pub_obj = rospy.Publisher('cmd_vel',Twist,queue_size=10)
       rate = rospy.Rate(10)
       base_pose_x = data.pose.pose.position.x
       base_pose_y = data.pose.pose.position.y
       coord_x = final_path[path_counter][0]
       coord_y = final_path[path_counter][1]
       next_coord_x = final_path[path_counter + 1][0]
       next_coord_y = final_path[path_counter + 1][1]
       #we should let the callback know that the next goal has been reached
       #while not rospy.is_shutdown():
       #get all the required data coordinates
       #get the present data coordinates
       #keep moving until the next coordinate is reached somewhat
       #while next coordinate is reached:
       base_z = data.pose.pose.position.z
       orient_x = data.pose.pose.orientation.x
       orient_y = data.pose.pose.orientation.y
       orient_z = data.pose.pose.orientation.z
       orient_w = data.pose.pose.orientation.w
       quaternion = (orient_x,orient_y,orient_z,orient_w)
       #convert the orientation to euler angles for us to work with
       euler_angles = euler_from_quaternion(quaternion)
       theta = euler_angles[2]
       print "theta",round(theta)
       theta = float(format(round(theta,2)))
       #now we have the orientation of the robot
       #loop through the current set of points
       coord_x = base_pose_x
       coord_y = base_pose_y
       slope = math.atan2(next_coord_y - coord_y, next_coord_x - coord_x)
       slope = round(slope)
       slope = float(format(round(slope,2)))
       print slope
       if theta != slope:
              if theta > slope:
                     twist_obj.angular.z = -0.1
                     #twist_obj.linear.x = 0
                     #twist_obj.linear.y = 0
                     #pub_obj.publish(twist_obj)
              elif theta < slope:
                     twist_obj.angular.z = 0.1
                     #twist_obj.linear.x = 0
                     #twist_obj.linear.y = 0
                     #pub_obj.publish(twist_obj)
              twist_obj.linear.x = 0
              twist_obj.linear.y = 0
              pub_obj.publish(twist_obj)
       elif theta == slope:
              twist_obj.linear.x = 0.3
              twist_obj.linear.y = 0
              twist_obj.angular.z = 0
              pub_obj.publish(twist_obj)
       print "next x",next_coord_x
       print "next y",next_coord_y
       print "b x",base_pose_x
       print "b y",base_pose_y
       if((next_coord_x - (base_pose_x) ) <= 0.5 and (next_coord_y - (base_pose_y)) <= 0.5):
              if(path_counter+1 < len(final_path)):
                     path_counter += 1
       #path_counter += 1
       rate.sleep()

def robot_move(final_path):
       #tell the robot to move to the destination
       #initialize the publisher object with its name and anonymous type
       #set the frequency in which to publish
       rospy.init_node('astar', anonymous=True)
       rospy.Subscriber('/base_pose_ground_truth',Odometry,callback)
       #callback(final_path)
       #initialize the publisher object with its name and anonymous type
       rospy.spin()

def get_all_neighbours(current):
       neighbours = []
       neighbour_calc = [(0,1),(1,0),(0,-1),(-1,0)]
       for i in range(0,4):
              #print (current[0]+neighbour_calc[i][0],current[1]+neighbour_calc[i][1])
              n = (current[0]+neighbour_calc[i][0],current[1]+neighbour_calc[i][1])
              neighbours.append(n)
       return neighbours

def get_all_distances(current,dest):
       distances = []
       for i in range(0,len(current)):
              #print (current[0]+neighbour_calc[i][0],current[1]+neighbour_calc[i][1])
              #calc the euclidean distance here
              d = math.sqrt(abs(((current[i][0] - dest[0])**2 + ((current[i][1] - dest[1])**2))))
              distances.append(d)
       return distances


def path_find(source, destination):
       #source = convert_coord_to_map(-8,-2)
       #destination = convert_coord_to_map(4.5,9)
       start = (-8,-2)
       final_path_list = []
       dest = (-5,5)
       #look at all the surrounding points from the source
       current = (-8,-2)
       #print current[0]
       parents = []
       final_path_list.append(current)
       while(current[0]<dest[0] and current[1]<dest[1]):
              #get all the neighbours into a list
              #convert the incoming point into an int
              int(current[0])
              int(current[1])
              #get_all_neighbours(current)
              #find out the euclidean distance between the neighbours and destination
              obstacle = 1
              neighbours = get_all_neighbours(current)
              revised_neighbours = []
              for i in neighbours:
                     if map[convert_coord_to_map(i[0],i[1])] == 0:
                            #print (i[0],i[1])
                            #print map[convert_coord_to_map(i[0],i[1])]
                            if i in parents:
                                   #do not add this point
                                   pass
                            else:
                                   revised_neighbours.append(i)
              distances = get_all_distances(revised_neighbours,dest)
              while obstacle == 1:
                     min_index = np.argmin(distances)
                     path_coord = revised_neighbours[min_index]
                     map_value =  convert_coord_to_map(revised_neighbours[min_index][0],revised_neighbours[min_index][1])
                     obstacle = map[map_value]
              final_path_list.append(path_coord)
              current = path_coord
              parents.append(current)
       print final_path_list
       c = 0
       for i in final_path_list:
              print i
              if (i[0]<=9 and i[0]>=-9) and (i[1]<=10 and i[1]>=-10):
                     final_path.append(i)
              if i == start:
                     del final_path[:]
                     final_path.append(start)
              else:
                     pass
              c = c+1

       print "path ultimate-->",final_path
       return final_path
       



def convert_map_to_coord(i):
       #convert the 
       x_coord = [-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9]
       y_coord = [10,9,8,7,6,5,4,3,2,1,0,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10]
       value = map[i]
       x_index = i % 18;
       y_index = i / 18;
       return (value,x_coord[x_index],y_coord[y_index])

def convert_coord_to_map(x,y):
       x_coord = [-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9]
       y_coord = [10,9,8,7,6,5,4,3,2,1,0,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10]

       if x>9:
              x = 9
       elif x<-9:
              x = -9

       if y>10:
              y = 10
       elif y<-10:
              y = -10


       x_int = int(x)
       y_int = int(y)

       x_index = x_coord.index(x_int)
       y_index = y_coord.index(y_int)

       map_value = x_index + 18*y_index
       return map_value


#create the main block
if __name__=='__main__':
       try:
              do_astar(map)
              
       except rospy.ROSInterruptException:
              pass