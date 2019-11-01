#!/usr/bin/env python
import moveit_commander

import sys
import rospy
import numpy as np
import math

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Vector3, Vector3Stamped

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from visualization_msgs.msg import *
from tf.broadcaster import TransformBroadcaster

import moveit_msgs.msg
import geometry_msgs.msg

from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs

import mpl_toolkits.mplot3d
import matplotlib.pyplot as pp

from pyquaternion import Quaternion

class MoveUR:
    def __init__(self):
        self.id_list = 0
        self.marker_publisher = rospy.Publisher(
            'visualization_marker', Marker, queue_size=5) 

        self.record_publisher = rospy.Publisher(
            'record_state', Int32, queue_size=1)

        rospy.Subscriber("/optoforce_node/optoforce_3D", geometry_msgs.msg.Vector3, self.optoforce_callback)
        rospy.Subscriber("output", PointCloud2, self.touch_points_callback)

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.manipulator = moveit_commander.MoveGroupCommander('manipulator')
        self.endeffector = moveit_commander.MoveGroupCommander('endeffector')

        self.manipulator.set_max_acceleration_scaling_factor(0.1)
        self.manipulator.set_max_velocity_scaling_factor(0.1)

        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        
        self.pose = self.manipulator.get_current_pose()

        print "============ Reference frame: %s" % self.manipulator.get_planning_frame()
        print "============ Reference frame: %s" % self.manipulator.get_end_effector_link()

        print "============ Robot Groups:"
        print self.robot.get_group_names()
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============"
        print "============ Printing robot position"
        print self.manipulator.get_current_pose()
        print "============"

        rospy.sleep(1)
        # rospy.spin()
        # according to https://en.wikipedia.org/wiki/List_of_common_coordinate_transformations#From_spherical_coordinates  ::: To Cartesian coordinate    From spherical coordinates


    def touch_points_callback(self, data): 
        self.point_data = data
    #     # print self.point_data


    def touch_point_transform(self):    

        self.x_points = []
        self.y_points = []  #list with points
        self.z_points = []

        self.t = geometry_msgs.msg.TransformStamped()
        
        self.q = quaternion_from_euler(0.8, 0.05, -2.65)

        #It would 

        
        for p in pc2.read_points(self.point_data, field_names = ("x", "y", "z"), skip_nans=True):
            #print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
            
            q1 = Quaternion(0, p[0],  p[1],  p[2])
            
            q2 = Quaternion(self.q[0],  self.q[1],  self.q[2], self.q[3])

            q3 = q1*q2

            #self.show_text_in_rviz(q3[1],q3[2],q3[2],3000+i)

            input_array =  np.array([p[0], p[1] , p[2]] )

            cloud_to_quaternion = Quaternion(vector=input_array) 

            rotated_array = q2.rotate(input_array)

            #Display the Points 

            # self.show_text_in_rviz(rotated_array[0]+0.575 ,rotated_array[1]+0.55,rotated_array[2]+0.68 ,1000+i)
            #self.show_text_in_rviz(p[0], p[1] , p[2] ,2000+i)
            #Appending the Coordinates of Touchpoints to the list
            self.x_points.append(rotated_array[0]+0.575)
            self.y_points.append(rotated_array[1]+0.55)
            self.z_points.append(rotated_array[2]+0.68)
            
        print "<<<<<working>>>>>"

        self.target_x_points = []
        self.target_y_points = []
        self.target_z_points = []
    
        self.target_points = []

        for i in range(0,len(self.x_points)):

            if(self.x_points[i] > 0.35 and self.x_points[i] < 0.45 and self.y_points[i] > 0.25 and self.y_points[i] < 0.35):
                    
                    self.show_text_in_rviz(self.x_points[i], self.y_points[i], self.z_points[i] ,1000+i)

                    self.target_x_points.append(self.x_points[i])
                    self.target_y_points.append(self.y_points[i])
                    self.target_z_points.append(self.z_points[i])
                    self.target_points.append([self.x_points[i],self.y_points[i],self.z_points[i]])

        # print self.target_x_points, 'AAAAAAAAAAAAAAAAAAAAAAA'


    def optoforce_callback(self, data):
        # self.optoforce_data = (data.x, data.y, data.z)
      
        f = pow(data.x, 2) + pow(data.y, 2) + pow(data.z, 2)
        self.force_data = math.sqrt(f)
 
        # print self.force_data
        # print data.x , data.y, data.z


    def show_text_in_rviz(self, x, y, z, id_num):
        r = float(id_num)/300
        marker = Marker(
            type=Marker.CUBE,
            id=self.id_list,
            lifetime=rospy.Duration(100),
            pose=geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y, z), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            scale=geometry_msgs.msg.Vector3(0.01, 0.01, 0.01),
            header=Header(frame_id='base_link'),
            color=std_msgs.msg.ColorRGBA(r, r, 0.0, 0.8),
        )
        self.id_list+=1
     
        self.marker_publisher.publish(marker)
        id1 = str(id_num)
        marker2 = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=self.id_list,
            lifetime=rospy.Duration(100),
            pose=geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y, z), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            scale=geometry_msgs.msg.Vector3(0.01, 0.01, 0.01),
            header=Header(frame_id='base_link'),
            color=std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 0.8),
            text = id1
        )
        self.id_list+=1
     
        self.marker_publisher.publish(marker2)


    def move_point(self, tar_point):
        self.manipulator.set_pose_target(tar_point)
        try:
            plan1 = self.manipulator.plan()
            self.manipulator.stop()
                    
            self.manipulator.execute(plan1)
            self.manipulator.stop()

        except:
            print("EERRRROOORRRR")
            rospy.sleep(0.1)
    

           
if __name__ == '__main__':

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR()

    ur.record_publisher.publish(0)

    rospy.sleep(1)

    ur.touch_point_transform()

    home = geometry_msgs.msg.Pose()
    home.position.x = float(0.304874925085)
    home.position.y = float(0.19950677174)
    home.position.z = float(0.445879639034) 

    """
    home.orientation.x = float(-0.246220984116)
    home.orientation.y = float(0.687315312725)
    home.orientation.z = float(0.247639520898) 
    home.orientation.w = float(0.636904667564)
    """
    home.orientation.x = float(-0.25615267)
    home.orientation.y = float(0.65907952)
    home.orientation.z = float(0.25615267) 
    home.orientation.w = float(0.65907952)
    # array([-0.25615267,  0.65907952,  0.25615267,  0.65907952])


    ur.move_point(home)

    # print ur.target_points

    # start_target = geometry_msgs.msg.Pose()
    # start_target.orientation = home.orientation

    # finish_target = geometry_msgs.msg.Pose()
    # finish_target.orientation = home.orientation


    # force = []

    # for i in range(4):
    # # for i in range(0,len(ur.target_x_points)):
    #     start_target.position.x = ur.target_points[i][0]
    #     start_target.position.y = ur.target_points[i][1]
    #     start_target.position.z = ur.target_points[i][2] + 0.05

        
    #     finish_target.position.x = ur.target_points[i][0]
    #     finish_target.position.y = ur.target_points[i][1]
    #     finish_target.position.z = ur.target_points[i][2] - 0.02

    #     # print start_target
    #     # print finish_target

    #     ur.move_point(start_target)
    #     ur.move_point(finish_target)
    #     rospy.sleep(0.1)

    #     tmp = 0
    #     print "=============================="
    #     # print "--------", ur.force_data, tmp

    #     while ur.force_data < 200 and tmp < 0.02:
    #         tmp += 0.0002
    #         finish_target.position.z = ur.target_points[i][2] - 0.02 - tmp
    #         ur.move_point(finish_target)
    #         rospy.sleep(0.1)

    #         print "########", ur.force_data, tmp
        

    #     finish_target.position.z = finish_target.position.z - 0.005

    #     ur.move_point(finish_target)
    #     rospy.sleep(2)

    #     force.append(ur.force_data)
    #     rospy.sleep(0.1)

    #     print "========", ur.force_data
    #     # print "!!!!!!!!!!!!!!!!!!!!!", force

    #     ur.move_point(start_target)
        

    # ur.move_point(home)
    
    # print "!!!!!!!!!!!!!!!!!!!!!", force

    # point_imformation = [ur.target_x_points, ur.target_y_points,ur.target_z_points, force]

    # np.save("point_force_small", np.array(point_imformation))


    # point_body = [ur.x_points, ur.y_points, ur.z_points]
    # np.save("point_body1", np.array(point_body))