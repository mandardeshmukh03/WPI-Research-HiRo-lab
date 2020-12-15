# -*- coding: utf-8 -*-
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
#from gazebo_msgs.msg import ModelState   
from gazebo_msgs.srv import GetModelState

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Models_state:

    _blockListDict = {
        'block_a': Block('new_table', 'link'),
        'block_b': Block('wood_cube_5cm', 'link'),
        'block_c': Block('wood_cube_5cm_0', 'link'),
        'block_d': Block('wood_cube_5cm_1', 'link'),


    }
    
    def show_gazebo_models(self):
        All_coordinate = []
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                if (block._name == 'new_table'):
                    table_pose_x = resp_coordinates.pose.position.x
                    table_pose_y = resp_coordinates.pose.position.y
                    table_pose_z = resp_coordinates.pose.position.z
                    table_coordinate = (table_pose_x,table_pose_y,table_pose_z)
                    table_rotation_z = resp_coordinates.pose.orientation.z
                    #print("table_coordinate : ")
                    #print(table_coordinate)
                    All_coordinate.append(table_rotation_z)
                    All_coordinate.append(table_coordinate)
                     
                if (block._name == 'wood_cube_5cm'):
                    cube1_pose_x = resp_coordinates.pose.position.x
                    cube1_pose_y = resp_coordinates.pose.position.y
                    cube1_pose_z = resp_coordinates.pose.position.z
                    cube1_coordinate = (cube1_pose_x,cube1_pose_y,cube1_pose_z)
                    #print("cube1_coordinate :")
                    #print(cube1_coordinate)
                    All_coordinate.append(cube1_coordinate)
                if (block._name == 'wood_cube_5cm_0'):
                    cube2_pose_x = resp_coordinates.pose.position.x
                    cube2_pose_y = resp_coordinates.pose.position.y
                    cube2_pose_z = resp_coordinates.pose.position.z
                    cube2_coordinate = (cube2_pose_x,cube2_pose_y,cube2_pose_z)
                    #print("cube2_coordinate :")
                    #print(cube2_coordinate)
                    All_coordinate.append(cube2_coordinate)
                if (block._name == 'wood_cube_5cm_1'):
                    cube3_pose_x = resp_coordinates.pose.position.x
                    cube3_pose_y = resp_coordinates.pose.position.y
                    cube3_pose_z = resp_coordinates.pose.position.z
                    cube3_coordinate = (cube3_pose_x,cube3_pose_y,cube3_pose_z)
                    #print("cube3_coordinate :")
                    #print(cube3_coordinate)
                    All_coordinate.append(cube3_coordinate)

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        return All_coordinate

states = Models_state()
coordinates = states.show_gazebo_models()
print(coordinates)
table_centroid = coordinates[0] 
table_orientation = (coordinates[1])*(0.018)  #converted into degrees.


#leng = 1600
#width = 1600

h1 = 0.8/math.cos(table_orientation)

x1_d = table_centroid[0]          #down side
y1_d = table_centroid[1] - h1
x1_u = table_centroid[0]          #up side
y1_u = table_centroid[1] + h1

x1_r = table_centroid[0] + h1      #right side
y1_r = table_centroid[1]
x1_l = table_centroid[0] - h1     #left side
y1_l = table_centroid[1]


def  line_eqn( table_orientation, x1, y1):
    m1 = math.tan(table_orientation)
    c1 = y1 - m1*x1
    return m1, c1

def line_eqn2(table_orientation, x1, y1):

    m0 = math.tan(table_orientation)
    m1 = -1/m0
    c1 = y1 - m1*x1
    return m1, c1

eqn_lin_d = line_eqn(table_orientation, x1_d, y1_d)   #down side line equation,thus describing  the rectangle.
eqn_lin_u = line_eqn(table_orientation, x1_u, y1_u)
eqn_lin_r = line_eqn2(table_orientation, x1_r, y1_r)
eqn_lin_l = line_eqn2(table_orientation, x1_l, y1_l)

m_n_c = [eqn_lin_d,eqn_lin_u, eqn_lin_r,eqn_lin_l]
#m_n_c.append(line_eqn(30, 2.4, 4.3))


def eucl_dis(point,m_n_c,table_orientation):    # point is centroid of object or all objects
    x = point[0]
    y = point[1]
    
    i = [0,1,2,3]
    distance = [0,0,0,0]
    for i in i:
        m_n_c[i]
        distance[i] = abs(y - m_n_c[i][0]*x -m_n_c[i][1])/math.sqrt(1+ m_n_c[i][0]*m_n_c[i][0])
        #print(m_n_c[i][1])
    #print(distance[1])
    min_dist_index =distance.index(min(distance))
    #print(min_dist_index)
    
    if(min_dist_index == 0):
        x_euc = x
        y_euc = y - distance[0]/math.cos(table_orientation) #- 10
        optimal_pos = (x_euc,y_euc)
    elif(min_dist_index == 1):
        x_euc = x
        y_euc = y + distance[1]/math.cos(table_orientation) #+ 10
        optimal_pos = (x_euc,y_euc)
    elif(min_dist_index == 2):
        x_euc = x + distance[2]/math.cos(table_orientation) # + 10
        y_euc = y 
        optimal_pos = (x_euc,y_euc)
    elif(min_dist_index == 3):
        x_euc = x - distance[3]/math.cos(table_orientation) #- 10
        y_euc = y 
        optimal_pos = (x_euc,y_euc)
    print('optimal_pos')
    print(optimal_pos)
    
    return optimal_pos
        
n = 3  # number of objects
if (n == 1):
        cube1_centroid = coordinates[2]  # centroid of object
        print('object_centroid')
        print(cube1_centroid)
        nearest_edge = eucl_dis(cube1_centroid, m_n_c,table_orientation)   #optimal position for single object

elif (n == 3):
    
    cube1 = coordinates[2]    # cube1 = (x,y,z)
    cube2 = coordinates[3]
    cube3 = coordinates[4]

    def distance(object1, object2):

        dis = math.sqrt((object1[0] - object2[0])**2  + (object1[1] - object2[1])**2 )

        return dis
        

    if (distance(cube1, cube2) <= 1800 and distance(cube1, cube3) <= 1800 and distance(cube2, cube3) <= 1800):
        Common_pointx = (cube1[0] + cube2[0] + cube3[0])/3
        Common_pointy = (cube1[1] + cube2[1] + cube3[1])/3 
        Common_point = (Common_pointx,Common_pointy)
        nearest_edge = eucl_dis(Common_point,m_n_c,table_orientation)   #optimal position for single object

 



    
