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
from gpis import MultiTaskGaussianProcessImplicitSurfaces, GaussianProcessImplicitSurfaces

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

# hyper parameter
alpha = 0.01
kernel_param = 0.6

class MoveUR5:   

    def __init__(self):

        self.HOST = '163.221.44.227'
        self.PORT = 30001
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))

        self.id_list = 0

        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10) 

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

    def force_callback(self, msg):
        # print msg
        if self.calib is None:
            self.calib = msg
        self.force = math.sqrt( (msg.x1 - self.calib.x1)**2 + (msg.x2 - self.calib.x2)**2 
                              + (msg.x3 - self.calib.x3)**2 + (msg.x4 - self.calib.x4)**2 
                              + (msg.x5 - self.calib.x5)**2 + (msg.x6 - self.calib.x6)**2 )

    def touch_points_callback(self, data): 
        self.point_data = data

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

            if z_points[i] > 0.03:     # 視覚データの制限
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

    def get_current_position(self):
        pose_position = ur.manipulator.get_current_pose().pose.position
        return np.array([pose_position.x, pose_position.y, pose_position.z])

    # socketで送信すると速度のx,yの向きが逆になるのでマイナスを付ける
    def send_speed_accel(self, speed, acceleration=0.1, t=0.1):
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

def plot_estimated_surface(position, var):
    N = var

    # ax.scatter(position[:,0], position[:,1], position[:,2], s=1000, c=N, cmap=cm.rainbow, linewidth=5);

    ax.plot_surface(position[0], position[1], position[2],facecolors=cm.rainbow(N),
    linewidth=0, rstride=1, cstride=1, antialiased=False, shade=False,vmin=N.min(), vmax=N.max())

    m = cm.ScalarMappable(cmap=cm.rainbow)
    m.set_array(N)
    # plt.colorbar(m)

def plot_path(position, i=None):

    plt.gca().patch.set_facecolor('white')
    ax._axis3don = False

    if i is None:
        ax.view_init(20, 60)
    else:
        ax.view_init(20, (i%360)*5)
    ax.set_xlim(0, 0.5)
    ax.set_ylim(0, 0.5)
    ax.set_zlim(-0.1, 0.4)

    start_po = position[0]
    current_po = position[-1]

    ax.plot(position[:,0], position[:,1], position[:,2],  "o-", color="black", ms=20, mew=5, linewidth=10)
    ax.plot([current_po[0]], [current_po[1]], [current_po[2]],  "o-", color="#00aa00", ms=30, mew=5)
    ax.plot([start_po[0]], [start_po[1]], [start_po[2]],  "o-", color="red", ms=30, mew=5)

def make_test_data():
    N = 40
    theta = np.linspace(-np.pi, np.pi, N)
    phi   = np.linspace(0, np.pi/2, N)
    r     = np.linspace(0.01, 0.3, N)

    THETA, PHI, R = np.meshgrid(theta, phi, r)

    X = R * np.sin(PHI) * np.cos(THETA) + 0.295668969809
    Y = R * np.sin(PHI) * np.sin(THETA) + 0.263545084525
    Z = R * np.cos(PHI) + 0.01

    x_test = np.ravel(X)[:,None]
    y_test = np.ravel(Y)[:,None]
    z_test = np.ravel(Z)[:,None]

    return X, Y, Z, np.concatenate([x_test, y_test, z_test], 1)


if __name__=="__main__":

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()
    print "force:", ur.force

    joint_home  = [-2.1336739699, -1.3189786116, -1.8041823546, -1.5889738242, 1.5709401369, 0.2687601149]
    # joint_start = [-2.1344168822, -1.4115842024, -2.1785548369, -1.1220162551, 1.570988059, 0.2703693509]
    joint_start = [-2.4498248736, -1.3869336287, -2.1113999526, -1.2148664633, 1.5698721409, -0.050427262] # line


    # move to home
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)

    # show body points on rviz
    ur.touch_point_transform()
    rospy.sleep(1)

    # move to start position
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_start) + "\n")
    rospy.sleep(3)



    # Visual data
    X1 = ur.target_points
    Y1 = np.zeros((np.shape(X1)[0], 1))
    T1 = 0

    # Tacticle data
    X2 =np.array([[0.33, 0.2, 0.25], [0.4, 0.3, 0.28], [0.1, 0.3, 0.23], [0.4, 0.4, 0.21]])
    Y2 = np.ones((np.shape(X2)[0], 1))
    T2 = 1
    
    # X3 =np.array([[0.23, 0.12, 0.22], [0.33, 0.21, 0.23], [0.15, 0.27, 0.28], [0.45, 0.34, 0.21]])
    # Y3 = np.ones((np.shape(X3)[0], 1))
    # X1 = np.concatenate([X1, X3])
    # Y1 = np.concatenate([Y1, Y3])

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
        if ur.force > 3:
            break
        ur.send_speed_accel([0,0,-0.005], t = 0.2)
        position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

        ######################################### Plot ###################################################################
        ax = fig.add_subplot(111, projection='3d')

        # ax.scatter(X1[:,0], X1[:,1], X1[:,2], s=100, color="green")
        plot_path(position_list)
        plt.draw()
        plt.pause(0.001)
        plt.clf()
        ######################################### Plot ###################################################################
    
    rospy.sleep(1)

    X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
    Y2 = np.append(Y2, [0])[:,None]
 
    error_list, var_ave_list = [], []
    for i in range(200):
            print "========================================================================="
            print "STEP: {}".format(i)

            if i == 0:
            # tmp = torch.Tensor([[10e-4, 10e-4], [10e-4, 10e-4]])
                task_kernel_params = torch.Tensor([[1, 10e-4], [10e-4, 1]])
            else:
                task_kernel_params = gp_model.task_kernel_params
                # kernel.params = gp_model.kernel.params
            print "-----------------------------------------------"
            # print task_kernel_params
            # print kernel.params

            X1_t = torch.from_numpy(X1).float()
            X2_t = torch.from_numpy(X2).float()
            Y1_t = torch.from_numpy(Y1).float()
            Y2_t = torch.from_numpy(Y2).float()

            gp_model = MultiTaskGaussianProcessImplicitSurfaces([X1_t, X2_t], [Y1_t, Y2_t], [T1, T2],
                                                    kernel, task_kernel_params, c=10, sigma=torch.tensor(-5.168), z_limit=0.03)
            # task_kernel_params = gp_model.task_kernel_params

            gp_model.learning()

            _, direrction = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4), T2)
            direrction = direrction.numpy().T[0]
            print "direction:", direrction
        

            ur.send_speed_accel(alpha * direrction, t=1)

            position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

            mean, var = gp_model.predict(XX, T2)
            estimated_surface, var = get_object_position(X, Y, Z, mean, var)
            var_ave_list.append(np.mean(var))
            # print [mean, var, gp_model.task_kernel]
            # np.save("../data/mtgpis/mean/step_{}".format(i), [mean, var, gp_model.task_kernel])

            ########################################## Plot ###################################################################
            ax = fig.add_subplot(111, projection='3d')
            plot_estimated_surface(estimated_surface, var)
            plot_path(position_list, i)
            # ax.scatter(X1[:,0], X1[:,1], X1[:,2], s=100, color="green")
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
            ########################################## Plot ###################################################################
            normal, _ = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4), T2)
            normal = normal.numpy().T[0]
            print "normal   :", normal

            interval = 100
            dt = normal / interval
            print "force:", ur.force

            if ur.force < 3:
                while True:
                    if ur.force > 3:
                        X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
                        Y2 = np.append(Y2, [0])[:,None]    
                        break        
                    ur.send_speed_accel(-dt, t=0.05)
                    rospy.sleep(0.05)

            else:
                X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
                Y2 = np.append(Y2, [0])[:,None]
                while True:
                    if ur.force < 3:
                        break
                    ur.send_speed_accel(dt/2, acceleration=0.1,t=0.05)
                    rospy.sleep(0.05)




    mean, var = gp_model.predict(XX, T2)
    estimated_surface, var = get_object_position(X, Y, Z, mean, var)
    var_ave_list.append(np.mean(var))
    position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

    # np.save("../data/mtgpis/position_400", position_list)
    # np.save("../data/mtgpis/error_300", error_list)
    # np.save("../data/mtgpis/var_400", var_ave_list)

    ########################################## Plot ###################################################################
    ax = fig.add_subplot(111, projection='3d')
    plot_estimated_surface(estimated_surface, var)
    plot_path(position_list)

    plt.show()
    ########################################## Plot ###################################################################


    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)
    print "-----THE END-----"

# def get_object_position(X_test, mean, var):
#     x = X_test[:,0]
#     y = X_test[:,1]
#     z = X_test[:,2]
#     mean = mean.T[0]
#     var  = var.T[0]

#     index =  np.where(np.abs(mean) < 0.005)
#     x = x[index][:,None]
#     y = y[index][:,None]
#     z = z[index][:,None]
#     var = var[index]
#     res = np.concatenate([x,y,z], 1)

#     return res, var

# def make_test_data():
#     N = 50

#     x = np.linspace(0.1, 0.6, N)
#     y = np.linspace(0, 0.5, N)
#     z = np.linspace(0.03, 0.2, N)
#     X, Y, Z = np.meshgrid(x,y,z)

#     x_test = np.ravel(X)[:,None]
#     y_test = np.ravel(Y)[:,None]
#     z_test = np.ravel(Z)[:,None]

#     return np.concatenate([x_test, y_test, z_test], 1)


    # task_kernel_params = torch.Tensor([[1, 10e-4], [10e-4, 1]])

    # X1_t = torch.from_numpy(X1).float()
    # X2_t = torch.from_numpy(X2).float()
    # Y1_t = torch.from_numpy(Y1).float()
    # Y2_t = torch.from_numpy(Y2).float()

    # gp_model = MultiTaskGaussianProcessImplicitSurfaces([X1_t, X2_t], [Y1_t, Y2_t], [T1, T2],
    #                                         kernel, task_kernel_params, c=20, z_limit=0.05)
    # gp_model.learning()

    # mean, var = gp_model.predict(XX, T1)
    # estimated_surface, var = get_object_position(X, Y, Z, mean, var)

    # ax = fig.add_subplot(111, projection='3d')
    # ax.view_init(45, 60)
    # ax.set_xlim(0, 0.5)
    # ax.set_ylim(0, 0.5)
    # ax.set_zlim(-0.1, 0.4)
    # plot_estimated_surface(estimated_surface, var)
    # plt.show()