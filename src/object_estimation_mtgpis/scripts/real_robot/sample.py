#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append("../")

import rospy
import socket
from std_msgs.msg import Int32, ColorRGBA
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
from supervisor.msg import Force

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
import time
import tf

# hyper parameter
alpha = 0.01
kernel_param = 0.2


class MoveUR5:   

    def __init__(self):

        self.HOST = '163.221.44.227'
        self.PORT = 30001
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))

        self.id_list = 0

        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1) 

        # rospy.Subscriber("/optoforce_node/optoforce_3D", geometry_msgs.msg.Vector3, self.optoforce_callback)
        rospy.Subscriber("output2", PointCloud2, self.touch_points_callback)
        rospy.Subscriber('/force', Force, self.force_callback)

        self.manipulator = moveit_commander.MoveGroupCommander('manipulator')

        self.manipulator.set_max_acceleration_scaling_factor(0.1)
        self.manipulator.set_max_velocity_scaling_factor(0.1)

        print "========== Printing robot position =========="
        print self.manipulator.get_current_pose()
        print "============================================="
        self.force = 0
        self.calib = None

        rospy.sleep(1)

    def touch_points_callback(self, data): 
        self.point_data = data

    def optoforce_callback(self, data):
        force_x = data.x + 20
        force_y = data.y - 2
        force_z = data.z - 238

        f = pow(force_x, 2) + pow(force_y, 2) + pow(force_z, 2)
        self.force_data = math.sqrt(f)

    def force_callback(self, msg):
        # print msg
        if self.calib is None:
            self.calib = msg
        self.force = math.sqrt( (msg.x1 - self.calib.x1)**2 + (msg.x2 - self.calib.x2)**2 
                              + (msg.x3 - self.calib.x3)**2 + (msg.x4 - self.calib.x4)**2 
                              + (msg.x5 - self.calib.x5)**2 + (msg.x6 - self.calib.x6)**2 )

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

            if z_points[i] > 0.11:     # 視覚データの制限
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
    def send_speed_accel(self, speed, acceleration=0.2, t=0.1):
        cmd = 'speedl([{}, {}, {}, 0, 0, 0], a={}, t={})\n'.format(-speed[0], -speed[1], speed[2], acceleration, t)
        ur.socket.send(cmd)
        rospy.sleep(0.01)



def get_object_position(x,y,z, mean, var):
    mean  = mean.reshape(x.shape)
    var   = var.reshape(x.shape)
    mean0 = np.argmin(np.abs(mean), axis = 2)

    mean0_x, mean0_y, mean0_z, var0, z_t = [], [], [], [], []
    for i in range(len(x)):
        for j in range(len(x)):
            mean0_x.append(x[i][j][mean0[i][j]])
            mean0_y.append(y[i][j][mean0[i][j]])
            mean0_z.append(z[i][j][mean0[i][j]])
            var0.append(var[i][j][mean0[i][j]])
            # tmp = np.sqrt(np.linalg.norm(r**2 - x[i][j][mean0[i][j]]**2 / 4 - y[i][j][mean0[i][j]]**2 / 4))
            # z_t.append(tmp)

    N = len(x)
    mean0_x = np.array(mean0_x).reshape((N, N))
    mean0_y = np.array(mean0_y).reshape((N, N))
    mean0_z = np.array(mean0_z).reshape((N, N))
    var0    = np.array(var0).reshape((N, N))
    # z_t     = np.array(z_t).reshape((N, N))

    # error   = np.sqrt(mean_squared_error(mean0_z, z_t))
    # var_ave = np.mean(var0)

    return [mean0_x, mean0_y, mean0_z], var0




    x = X_test[:,0]
    y = X_test[:,1]
    z = X_test[:,2]
    mean = mean.T[0]
    var  = var.T[0]

    index =  np.where(np.abs(mean) < 0.005)
    x = x[index][:,None]
    y = y[index][:,None]
    z = z[index][:,None]
    var = var[index]
    res = np.concatenate([x,y,z], 1)

    return res, var

def plot_estimated_surface(position, var):
    N = var

    # ax.scatter(position[:,0], position[:,1], position[:,2], s=500, c=N, cmap=cm.rainbow, linewidth=5);
    # ax.plot_trisurf(position[:,0], position[:,1], position[:,2])
    ax.plot_surface(position[0], position[1], position[2],facecolors=cm.rainbow(N),
    linewidth=0, rstride=1, cstride=1, antialiased=False, shade=False)

    m = cm.ScalarMappable(cmap=cm.rainbow)
    m.set_array(N)
    # plt.colorbar(m)

def plot_path(position):

    # plt.gca().patch.set_facecolor('white')
    # ax._axis3don = False

    # ax.view_init(70, 20)
    ax.set_xlim(0, 0.5)
    ax.set_ylim(0, 0.5)
    ax.set_zlim(-0.1, 0.4)

    start_po = position[0]
    current_po = position[-1]

    ax.plot(position[:,0], position[:,1], position[:,2],  "o-", color="black", ms=20, mew=5, linewidth=10)
    ax.plot([current_po[0]], [current_po[1]], [current_po[2]],  "o-", color="#00aa00", ms=30, mew=5)
    ax.plot([start_po[0]], [start_po[1]], [start_po[2]],  "o-", color="red", ms=30, mew=5)

def make_test_data():
    N = 50

    x = np.linspace(0.1, 0.6, N)
    y = np.linspace(0, 0.5, N)
    z = np.linspace(0.03, 0.2, N)
    X, Y, Z = np.meshgrid(x,y,z)

    x_test = np.ravel(X)[:,None]
    y_test = np.ravel(Y)[:,None]
    z_test = np.ravel(Z)[:,None]

    return np.concatenate([x_test, y_test, z_test], 1)

def make_test_data():
    N = 40
    theta = np.linspace(-np.pi, np.pi, N)
    phi   = np.linspace(0, np.pi/2, N)
    r     = np.linspace(0.01, 0.25, N)

    THETA, PHI, R = np.meshgrid(theta, phi, r)

    X = R * np.sin(PHI) * np.cos(THETA) + 0.3367
    Y = R * np.sin(PHI) * np.sin(THETA) + 0.195
    Z = R * np.cos(PHI)

    x_test = np.ravel(X)[:,None]
    y_test = np.ravel(Y)[:,None]
    z_test = np.ravel(Z)[:,None]

    return X, Y, Z, np.concatenate([x_test, y_test, z_test], 1)



if __name__=="__main__":

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()
    print "force:", ur.force

    joint_home  = [-2.2574303786, -1.2912290732, -1.5104644934, -1.9099796454, 1.5702439547, 0.1433893889]
    # joint_start = [-2.4498248736, -1.3869336287, -2.1113999526, -1.2148664633, 1.5698721409, -0.050427262] # line
    joint_start = [-2.3323529402, -1.4033992926, -2.209312741, -1.0986922423, 1.5712637901, 0.0724981949]

    # move to home
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)

    # move to start position
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_start) + "\n")
    rospy.sleep(3)

    X1 = np.array([[0.33, 0.2, 0.2], [0.4, 0.1, 0.18], [0.1, 0.3, 0.19], [0.4, 0.4, 0.21]])
    Y1 = np.array([[1], [1], [1], [1]])

    # test data
    # X_test = make_test_data()
    # XX = torch.from_numpy(X_test).float()

    X, Y, Z, XX = make_test_data()
    XX = torch.from_numpy(XX).float()

    kernel = InverseMultiquadricKernelPytouch([kernel_param])

    # Show environment
    fig = plt.figure(figsize=(40, 40), dpi=50)

    position_list = np.array([ur.get_current_position()])

    while True:
        print "force:", ur.force
        if ur.force > 5:
            break
        ur.send_speed_accel([0,0,-0.003], t = 0.5)
        position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

        ######################################### Plot ###################################################################
        ax = fig.add_subplot(111, projection='3d')
        plot_path(position_list)
        plt.draw()
        plt.pause(0.001)
        plt.clf()
        ######################################### Plot ###################################################################
    
    rospy.sleep(1)

    X1 = np.append(X1, ur.get_current_position()[:,None].T, axis=0)
    Y1 = np.append(Y1, [0])[:,None]
 
    # error_list, var_ave_list = [], []
    
    for i in range(200):
            print "========================================================================="
            print "STEP: {}".format(i)
            print "current position:", ur.get_current_position()
            X_t = torch.from_numpy(X1).float()
            Y_t = torch.from_numpy(Y1).float()

            gp_model = GaussianProcessImplicitSurfaces(X_t, Y_t, kernel, c=20, z_limit=0.08)

            _, direrction = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4))
            direrction =  direrction.numpy().T[0]
            print "direction:", direrction
            
            s = time.time()

            ur.send_speed_accel(alpha * direrction, t=0.5)

            position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

            mean, var = gp_model.predict(XX)
            estimated_surface, var = get_object_position(X,Y,Z, mean, var)

            # ########################################## Plot ###################################################################
            ax = fig.add_subplot(111, projection='3d')
            plot_estimated_surface(estimated_surface, var)
            plot_path(position_list)
            plt.draw()
            plt.pause(0.001)
            plt.clf()
            # ########################################## Plot ###################################################################
       
            normal,_ = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4))
            normal = normal.numpy().T[0]
            print "normal   :", normal
            # print create_orientation(normal)

            interval = 100
            dt = normal / interval
            print "force:", ur.force

            print "time:", time.time() - s

            if ur.force < 5:
                while True:
                    if ur.force > 5:
                        X1 = np.append(X1, ur.get_current_position()[:,None].T, axis=0)
                        Y1 = np.append(Y1, [0])[:,None]    
                        break        
                    ur.send_speed_accel(-dt, t=0.1)
                    rospy.sleep(0.1)

            else:
                X1 = np.append(X1, ur.get_current_position()[:,None].T, axis=0)
                Y1 = np.append(Y1, [0])[:,None]
                while True:
                    if ur.force < 5:
                        break
                    ur.send_speed_accel(dt/2, acceleration=0.1,t=0.1)
                    rospy.sleep(0.1)



    position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

    mean, var = gp_model.predict(XX)
    estimated_surface, var = get_object_position(X,Y,Z, mean, var)


    # mean, var = gp_model.predict(XX)
    # estimated_surface, var, error, var_ave = get_object_position(X, Y, Z, mean, var, radius)

    # error_list.append(error)
    # var_ave_list.append(var_ave)
    # position_list = np.append(position_list, [current_po[0]], axis = 0)

    # np.save("../data/gpis_200_per5/gpis_position", position_list)
    # np.save("../data/gpis_error_200", error_list)
    # np.save("../data/gpis_var_ave_200", var_ave_list)

    # ########################################## Plot ###################################################################
    ax = fig.add_subplot(111, projection='3d')
    # plot_environment(inside_surface, outside_surface)
    plot_estimated_surface(estimated_surface, var)
    plot_path(position_list)

    plt.show()
    # ########################################## Plot ###################################################################



    # # if i % 50 == 0 and i > 0:
    # #     mean, var = gp_model.predict(XX)
    # #     mean_x, mean_y, mean_z, var, error, var_ave = get_object_position(X, Y, Z, mean, var, radius)
    # #     np.save("../data/gpis_200_per5/gpis_{}".format(i), [mean_x, mean_y, mean_z, var, error, var_ave])


    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)
    print "-----THE END-----"