#!/usr/bin/env python
# coding: utf-8

import sys
sys.dont_write_bytecode = True


import moveit_commander
import rospy
import socket

import numpy as np
import math

from std_msgs.msg import Int32, ColorRGBA
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import moveit_msgs.msg


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker

from gpis import GaussianProcessImplicitSurface
from dmp_discrete import DMPs_discrete
from calculate_hesse import eigenvalue_of_hesse_matrix

# from Quaternion import Quat

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class MoveUR5:   

    def __init__(self):

        self.HOST = '163.221.44.227'
        self.PORT = 30001
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))

        self.id_list = 0

        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1) 

        rospy.Subscriber("/optoforce_node/optoforce_3D", geometry_msgs.msg.Vector3, self.optoforce_callback)
        rospy.Subscriber("output", PointCloud2, self.touch_points_callback)

        self.manipulator = moveit_commander.MoveGroupCommander('manipulator')

        self.manipulator.set_max_acceleration_scaling_factor(0.1)
        self.manipulator.set_max_velocity_scaling_factor(0.1)

        print "========== Printing robot position =========="
        print self.manipulator.get_current_pose()
        print "============================================="

        rospy.sleep(1)

    def touch_points_callback(self, data): 
        self.point_data = data

    def optoforce_callback(self, data):
        force_x = data.x + 18
        force_y = data.y - 5
        force_z = data.z - 262

        f = pow(force_x, 2) + pow(force_y, 2) + pow(force_z, 2)
        self.force_data = math.sqrt(f)

    def touch_point_transform(self):    

        x_points = []
        y_points = [] 
        z_points = []

        for p in pc2.read_points(self.point_data, field_names = ("x", "y", "z"), skip_nans=True):
            x_points.append(p[0])
            y_points.append(p[1])
            z_points.append(p[2])

        print "<<<<<working>>>>>"

        self.target_x_points = []
        self.target_y_points = []
        self.target_z_points = []
        self.target_points   = []

        for i in range(0,len(x_points)):

            if z_points[i] > 0.11:     
                    self.show_text_in_rviz(x_points[i], y_points[i], z_points[i] ,1000+i)

                    self.target_x_points.append(x_points[i])
                    self.target_y_points.append(y_points[i])
                    self.target_z_points.append(z_points[i])
                    self.target_points.append([x_points[i],y_points[i],z_points[i]])
    
        self.target_points = np.reshape(self.target_points,(-1,3))

    def show_text_in_rviz(self, x, y, z, id_num):
        r = float(id_num) / 300
        marker = Marker(
            type     = Marker.CUBE,
            id       = self.id_list,
            lifetime = rospy.Duration(100),
            pose     = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y, z), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            scale    = geometry_msgs.msg.Vector3(0.01, 0.01, 0.01),
            header   = Header(frame_id='kinect2_link'),
            color    = ColorRGBA(r, r, 0.0, 0.8),
        )
        self.id_list += 1
     
        self.marker_publisher.publish(marker)

        id1 = str(id_num)
        marker2 = Marker(
            type     = Marker.TEXT_VIEW_FACING,
            id       = self.id_list,
            lifetime = rospy.Duration(100),
            pose     = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x, y, z), geometry_msgs.msg.Quaternion(0, 0, 0, 1)),
            scale    = geometry_msgs.msg.Vector3(0.01, 0.01, 0.01),
            header   = Header(frame_id='kinect2_link'),
            color    = ColorRGBA(0.0, 1.0, 0.0, 0.8),
            text     = id1
        )
        self.id_list += 1
     
        self.marker_publisher.publish(marker2)
        rospy.sleep(0.001)

    def move_point(self, tar_point):
        self.manipulator.set_pose_target(tar_point)
        try:
            plan1 = self.manipulator.plan()
            # self.manipulator.stop()
                    
            self.manipulator.execute(plan1)
            # self.manipulator.stop()

        except:
            print("EERRRROOORRRR")
            rospy.sleep(0.01)

    def pose_position_to_list_position(self,pose_position):
        return np.array([pose_position.x, pose_position.y, pose_position.z])


    # socketで送信すると速度のx,yの向きが逆になるのでマイナスを付ける
    def send_speed_accel(self, speed, acceleration=0.2, t=0.5):
        cmd = 'speedl([{}, {}, {}, 0, 0, 0], a={}, t={})\n'.format(-speed[0], -speed[1], speed[2], acceleration, t)
        ur.socket.send(cmd)
        rospy.sleep(0.1)


           
if __name__ == '__main__':

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()

    joint_home  = [-2.2574303786, -1.2912290732, -1.5104644934, -1.9099796454, 1.5702439547, 0.1433893889]
    # joint_start = [-2.4558013121, -1.4650853316, -2.054459397, -1.1939886252, 1.572055459, -0.0557101409]  # circle
    joint_start = [-2.4498248736, -1.3869336287, -2.1113999526, -1.2148664633, 1.5698721409, -0.050427262] # line

    # move to home
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)

    # show body points on rviz
    ur.touch_point_transform()
    rospy.sleep(1)

    # read orbit data and generate GP data 
    orbit_position = np.load("data/circle_orbit.npy")
    # orbit_position = np.load("data/line_orbit.npy")

    X = ur.target_points
    Y = np.zeros((np.shape(X)[0], 1))

    # print orbit_position
    # print "target:", len(ur.target_points)

    # Generate DMPs 
    dmp = DMPs_discrete(dmps=3, bfs=100, dt= 0.01)
    dmp.imitate_path(y_des=np.array([orbit_position[:,0], orbit_position[:,1], orbit_position[:,2]]))

    # Generate GPIS
    gpis = GaussianProcessImplicitSurface(X, Y)

    # move to start position
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_start) + "\n")
    rospy.sleep(3)


    # y_track = [] 
    # y1_track = [] 

    # for i in range(dmp.timesteps):
    #     current_position =  ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)
    #     y, dy, ddy = dmp.step(tau=1)

    #     print "current:", current_position
    #     print "y:",y
    #     print "dy:",dy
    #     print "ddy:", np.linalg.norm(ddy)

    #     send_dy = 0.095 * dy
    #     send_ddy = 0.1 * np.linalg.norm(ddy)

    #     ur.send_speed_accel(send_dy, send_ddy)

    #     y_track.append(ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position))
    #     y1_track.append(np.copy(y))

    # y_track = np.array(y_track)
    # y1_track = np.array(y1_track)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(orbit_position[:,0], orbit_position[:,1], orbit_position[:,2])
    # ax.scatter(y_track[:,0], y_track[:,1], y_track[:,2])
    # ax.scatter(y1_track[:,0], y1_track[:,1], y1_track[:,2])

    # plt.show()

    while True:
        print "force:", ur.force_data
        if ur.force_data > 100:
            break
        ur.send_speed_accel([0,0,-0.005])
  

    y_track = []
    dy_track = []
    dy_norm = []
    step = 1
    current_position =  ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)
    

    while (np.linalg.norm(current_position[:2] - dmp.goal[:2]) > 10e-3) or (step < 50):
        current_position = ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)
        y, dy, ddy = dmp.step(tau=1)

        n,d = gpis.direction_func(current_position, y)

        interval = 400
        dt = n / interval

        print "force1:", ur.force_data

        if ur.force_data < 500:
            while True:
                if ur.force_data > 500:
                    break        
                ur.send_speed_accel(-dt)

        print "force2:", ur.force_data
        if ur.force_data > 10000:
            while True:
                if ur.force_data < 5000:
                    break
                ur.send_speed_accel(dt)


        current_position = ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)
        y_track.append(current_position)

        input_data_to_gp = X[np.where((X[:, 0] > current_position[0]-0.03)\
                                    & (X[:, 0] < current_position[0]+0.03)\
                                    & (X[:, 1] > current_position[1]-0.03)\
                                    & (X[:, 1] < current_position[1]+0.03))]

        eigenvalue = eigenvalue_of_hesse_matrix(input_data_to_gp)
        # print "input_data_to_gp:", len(input_data_to_gp)
        # print "eigenvalue:", eigenvalue

        n,d = gpis.direction_func(current_position, y)

        # n,d = gpis.direction_func(current_position, y, data_number=len(input_data_to_gp))

        if eigenvalue[0] > 0 or eigenvalue[1] > 0:
            d = 0.1 * n + d
            d /= np.linalg.norm(d)
        
        direction = 0.05 * np.linalg.norm(dy) * d

        # print "direction:", direction
        # print "dy:",np.linalg.norm(dy)

        ur.send_speed_accel(direction)


        current_position = ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)

        dy_track.append(np.copy(dy))
        y_track.append(current_position)
        dy_norm.append(np.linalg.norm(dy))

        step += 1

    y_track = np.array(y_track)
    dy_track = np.array(dy_track)
    dy_norm = np.array(dy_norm)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(orbit_position[:,0], orbit_position[:,1], orbit_position[:,2])
    ax.scatter(y_track[:,0], y_track[:,1], y_track[:,2])
    # ax.scatter(y1_track[:,0], y1_track[:,1], y1_track[:,2])

    # plt.show()

    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)
    print "-----THE END-----"
    


    # show body points on plt
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(ur.target_points[:, 0], ur.target_points[:, 1], ur.target_points[:, 2])
    # plt.show()