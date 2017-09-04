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
import random
import math

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

def do_astar(pub_obj,data):
       #print len(map)
       #now find a path from (-8,-2) to (4.5,9)
       source = convert_coord_to_map(-8,-2)
       destination = convert_coord_to_map(4.5,9)
       #print map[source]
       #print map[destination]
       #now find the path from the source to the destination
       path_find(source, destination)

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
       final_path_list = []
       dest = (4.5,9)
       #look at all the surrounding points from the source
       current = (-8,-2)
       #print current[0]



       #the calculated neighbours must not be an obstacle
       #get the final value if map[map_value] == 0
       #continue
       #else
       #add a 100 to this distance and calculate the min one in the array again
       #distances[min_index] = distances[min_index] + 100
       #now do argmin again and then verify
       #only if the final map value is 0
       #add it to the 
       parents = []
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

              print revised_neighbours
              distances = get_all_distances(revised_neighbours,dest)
              while obstacle == 1:
                     min_index = np.argmin(distances)
                     path_coord = revised_neighbours[min_index]
                     #print path_coord
                     map_value =  convert_coord_to_map(revised_neighbours[min_index][0],revised_neighbours[min_index][1])
                     obstacle = map[map_value]
                     #if obstacle == 1:
                            #distances[min_index] = distances[min_index] + 100
              #add the coordinate to the final_path_list
              final_path_list.append(path_coord)
              current = path_coord
              parents.append(current)
       print final_path_list


def convert_map_to_coord(i):
       #convert the 
       x_coord = [-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9]
       y_coord = [10,9,8,7,6,5,4,3,2,1,0,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10]
       #modulo by 18 gives the x  value
       # div by 20 gives the y  value
       #print map[i]
       value = map[i]
       x_index = i % 18;
       y_index = i / 18;
       print (x_coord[x_index],y_coord[y_index])
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
              rospy.init_node('robot_astar',anonymous=True)
              #create a publisher publishing velocities to the robot
              pub_obj = rospy.Publisher('cmd_vel',Twist,queue_size=10)
              do_astar(pub_obj,map)
              rospy.spin()
       except rospy.ROSInterruptException:
              pass