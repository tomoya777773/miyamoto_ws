#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append("../")
sys.dont_write_bytecode = True

import rospy
import socket
from std_msgs.msg import Int32, ColorRGBA
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker


from kernel import InverseMultiquadricKernelPytouch
from gpis import GaussianProcessImplicitSurfaces

import numpy as np
import math
import torch

import matplotlib.pyplot as plt

from matplotlib.colors import BoundaryNorm
from matplotlib.ticker import MaxNLocator
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from sklearn.metrics import mean_squared_error
# import time


# hyper parameter
alpha = 0.02
kernel_param = 0.3


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

            if z_points[i] > 0.05:     
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

    def get_current_position(self):
        pose_position = ur.manipulator.get_current_pose().pose.position
        return np.array([pose_position.x, pose_position.y, pose_position.z])

    # socketで送信すると速度のx,yの向きが逆になるのでマイナスを付ける
    def send_speed_accel(self, speed, acceleration=0.2, t=0.5):
        cmd = 'speedl([{}, {}, {}, 0, 0, 0], a={}, t={})\n'.format(-speed[0], -speed[1], speed[2], acceleration, t)
        ur.socket.send(cmd)
        rospy.sleep(0.1)



def get_object_position(X_test, mean, var):
    x = X_test[:,0]
    y = X_test[:,1]
    z = X_test[:,2]
    mean = mean.T[0]
    var  = var.T[0]

    index =  np.where(np.abs(mean) < 0.01)
    x = x[index][:,None]
    y = y[index][:,None]
    z = z[index][:,None]
    var = var[index]
    res = np.concatenate([x,y,z], 1)

    return res, var


def plot_estimated_surface(position, var):
    plt.gca().patch.set_facecolor('white')
    ax._axis3don = False

    ax.view_init(60, 0)
    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-0.4, 0.4)
    ax.set_zlim(-0.1, 0.3)

    N = var
    ax.scatter(position[:,0], position[:,1], position[:,2], s=400, c=N, cmap=cm.rainbow, linewidth=5);
    # ax.plot_trisurf(position[:,0], position[:,1], position[:,2], cmap='viridis', edgecolor='none');
    # ax.plot_surface(position[0], position[1], position[2],facecolors=cm.rainbow(N),
    # linewidth=0, rstride=1, cstride=1, antialiased=False, shade=False)

    # m = cm.ScalarMappable(cmap=cm.rainbow)
    # m.set_array(N)
    # plt.colorbar(m)


def plot_environment(in_surf, out_surf):


    ax.plot_surface(in_surf[0], in_surf[1], in_surf[2], color="green",
    rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False, alpha=0.1)
    ax.plot_surface(out_surf[0], out_surf[1], out_surf[2], color="blue",
    rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False, alpha=0.1)

def make_test_data():
    N = 50

    x = np.linspace(0, 0.5, N)
    y = np.linspace(0, 0.5, N)
    z = np.linspace(0.05, 0.2, N)
    X, Y, Z = np.meshgrid(x,y,z)

    x_test = np.ravel(X)[:,None]
    y_test = np.ravel(Y)[:,None]
    z_test = np.ravel(Z)[:,None]

    return np.concatenate([x_test, y_test, z_test], 1)


if __name__=="__main__":

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()

    # show body points on rviz
    ur.touch_point_transform()
    rospy.sleep(1)

    X = ur.target_points
    Y = np.zeros((np.shape(X)[0], 1))

    X1 = np.array([[0.4, 0.4, 0.3],[0.2, 0.3, 0.2], [0.1, 0.5, 0.3], [0.5, 0.3, 0.2]])
    Y1 = np.array([[1], [1], [1], [1]])
    X  = np.append(X, X1, axis=0)
    Y  = np.append(Y, Y1)[:,None]

    X_t = torch.from_numpy(X).float()
    Y_t = torch.from_numpy(Y).float()
    
    # test data
    X_test = make_test_data()
    XX = torch.from_numpy(X_test).float()

    kernel   = InverseMultiquadricKernelPytouch([kernel_param])
    gp_model = GaussianProcessImplicitSurfaces(X_t, Y_t, kernel, sigma=0.2, c=20, z_limit=0.1)
    # gp_model.learning()

    mean, var = gp_model.predict(XX)

    estimated_surface, var = get_object_position(X_test, mean, var)

    # estimated_surface, var, error, var_ave = get_object_position(X, Y, Z, mean, var)
    # print "error:", error
    # error_list.append(error)
    # var_ave_list.append(var_ave)

    # ########################################## Plot ###################################################################
    fig = plt.figure(figsize=(40, 40), dpi=50)

    ax = fig.add_subplot(111, projection='3d')

    # plot_environment(inside_surface, outside_surface)
    plot_estimated_surface(estimated_surface, var)
    plt.show()
    # ########################################## Plot ###################################################################


