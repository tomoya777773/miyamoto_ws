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
alpha        = 0.01
kernel_param = 0.2
sigma        = torch.tensor(-3.1)
# sigma        = torch.tensor(-5.168)

max_iter     = 50
lr           = 0.0005

plot = True
# plot = False
save_data = True
save_data = False
save_movie = True
save_movie = False


class MoveUR5:   

    def __init__(self):

        self.HOST = '163.221.44.227'
        self.PORT = 30001
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))

        self.id_list = 0

        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10) 

        rospy.Subscriber("output", PointCloud2, self.touch_points_callback1)
        rospy.Subscriber("output2", PointCloud2, self.touch_points_callback2)

        # rospy.Subscriber('/force', Force, self.force_callback)
        rospy.Subscriber("/optoforce_node/optoforce_3D", geometry_msgs.msg.Vector3, self.optoforce_callback)

        self.manipulator = moveit_commander.MoveGroupCommander('manipulator')
        self.manipulator.set_max_acceleration_scaling_factor(0.1)
        self.manipulator.set_max_velocity_scaling_factor(0.1)

        print "========== Printing robot position =========="
        print self.manipulator.get_current_pose()
        print "============================================="
        self.force = 0
        self.calib = None

        rospy.sleep(1)

    def optoforce_callback(self, data):
        force_x = data.x + 15
        force_y = data.y - 5
        force_z = data.z - 300

        f = pow(force_x, 2) + pow(force_y, 2) + pow(force_z, 2)
        self.force = math.sqrt(f)

    def touch_points_callback1(self, data): 
        self.point_data1 = data

    def touch_points_callback2(self, data): 
        self.point_data2 = data

    def touch_point_transform1(self):    
        x_points = []
        y_points = [] 
        z_points = []
        target_points = []
        for p in pc2.read_points(self.point_data1, field_names = ("x", "y", "z"), skip_nans=True):
            x_points.append(p[0])
            y_points.append(p[1])
            z_points.append(p[2])
            target_points.append([p[0], p[1], p[2]])

        self.target_points1 = np.reshape(target_points,(-1,3))

    def touch_point_transform2(self):    

        x_points = []
        y_points = [] 
        z_points = []

        for p in pc2.read_points(self.point_data2, field_names = ("x", "y", "z"), skip_nans=True):
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
    def send_speed_accel(self, speed, acceleration=0.2, t=0.5):
        cmd = 'speedl([{}, {}, {}, 0, 0, 0], a={}, t={})\n'.format(-speed[0], -speed[1], speed[2], acceleration, t)
        ur.socket.send(cmd)
        rospy.sleep(t)


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

    ax.plot_surface(position[0], position[1], position[2],facecolors=cm.rainbow(N), alpha=1,
    linewidth=0, rstride=1, cstride=1, antialiased=False, shade=False,vmin=N.min(), vmax=N.max())

    m = cm.ScalarMappable(cmap=cm.rainbow)
    m.set_array(N)
    # plt.colorbar(m)

def plot_path(position, i=None):

    plt.gca().patch.set_facecolor('white')
    ax._axis3don = False

    ax.view_init(60,-45)
    # if i is None:
    #     ax.view_init(20, 60)
    # else:
    #     ax.view_init(20, (i%360)*5)
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
    r     = np.linspace(0.05, 0.4, N)

    THETA, PHI, R = np.meshgrid(theta, phi, r)

    X = R * np.sin(PHI) * np.cos(THETA) + 0.304797149876
    Y = R * np.sin(PHI) * np.sin(THETA) + 0.150673962939
    # Z = R * np.cos(PHI) + 0.01
    Z = R * np.cos(PHI) + 0.1

    x_test = np.ravel(X)[:,None]
    y_test = np.ravel(Y)[:,None]
    z_test = np.ravel(Z)[:,None]

    return X, Y, Z, np.concatenate([x_test, y_test, z_test], 1)

def create_dummpy_visual(X1, Y1):
    X1_t = torch.from_numpy(X1).float()
    Y1_t = torch.from_numpy(Y1).float()

    kernel = InverseMultiquadricKernelPytouch([kernel_param])
    gp_model = GaussianProcessImplicitSurfaces(X1_t, Y1_t, kernel,sigma=sigma , c=10, z_limit=0.03)

    normal_list = []
    for i in range(len(X1)):
        normal,_ = gp_model.predict_direction(X1_t[i][:, None].T)
        normal = normal.numpy().T[0]
        normal_list.append(normal)
    normal_list = np.array(normal_list).reshape((-1,3))

    X1 = np.concatenate([X1, X1 + normal_list*0.02], 0)
    Y1 = np.concatenate([Y1, np.ones(np.shape(Y1)[0])[:, None]], 0)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(X1[:,0],X1[:,1],X1[:,2])
    # plt.show()

    return X1, Y1


if __name__=="__main__":

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()
    print "force:", ur.force

    joint_home  = [-2.1336739699, -1.3189786116, -1.8041823546, -1.5889738242, 1.5709401369, 0.2687601149]
    # joint_start = [-2.3925276438, -1.3441069762, -2.0282152335, -1.3387392203, 1.5715278387, 0.0112262229] # line
    joint_start = [-2.286190335, -1.3497789542, -1.9421065489, -1.4196661154, 1.5712277889, 0.1171049774] # 似ていない


    # move to home
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)

    # show body points on rviz
    ur.touch_point_transform1()
    ur.touch_point_transform2()
    rospy.sleep(1)

    # move to start position
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_start) + "\n")
    rospy.sleep(3)

    # get visual data
    X1 = ur.target_points
    Y1 = np.zeros((np.shape(X1)[0], 1))
    X1, Y1 = create_dummpy_visual(X1, Y1)
    T1 = 0
    
    # body data
    X_body = ur.target_points1
    # X_body[:, 2] -= 0.01

    # test data
    X, Y, Z, XX = make_test_data()
    XX = torch.from_numpy(XX).float()

    # Show environment
    fig = plt.figure(figsize=(40, 40), dpi=50)

    position_list = np.array([ur.get_current_position()])
 
    movie_num = 1
    while True:
        print "force:", ur.force
        if ur.force > 50:
            break
        ur.send_speed_accel([0,0,-0.01],acceleration=1, t = 0.1)
        position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

        ######################################### Plot ###################################################################
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(X_body[:,0], X_body[:,1], X_body[:,2], s=200, color="navajowhite", alpha=1)
        # ax.scatter(X1[:,0], X1[:,1], X1[:,2], s=100, color="green")
        plot_path(position_list)
        # plt.savefig('../data/body/mtgpis/movie/step{}.png'.format(movie_num))
        movie_num += 1
        plt.draw()
        plt.pause(0.001)
        plt.clf()
        ######################################### Plot ###################################################################
    
    rospy.sleep(1)

    # Tacticle data
    X2 = np.array([ur.get_current_position()])
    Y2 = np.array([[0]])
    X2 = np.append(X2, [ur.get_current_position() + np.array([0,0,0.02])], axis=0)
    Y2 = np.append(Y2, [1])[:,None]
    T2 = 1

    kernel = InverseMultiquadricKernelPytouch([kernel_param])


    var_list = []
    simirality_list = []

    for i in range(150):
        print "========================================================================="
        print "STEP: {}".format(i)

        if i == 0:
            # task_kernel_params = torch.Tensor([[1, 10e-4], [10e-4, 1]])
            # task_kernel_params = torch.tensor([[-0.2881, -0.3729], [0.27, -1.354]])
            task_kernel_params = torch.tensor([[-0.2458, -1.8297], [0.27, 0.1005]])

        else:
            task_kernel_params = gp_model.task_kernel_params
        print "-----------------------------------------------"
        print task_kernel_params

        X1_t = torch.from_numpy(X1).float()
        X2_t = torch.from_numpy(X2).float()
        Y1_t = torch.from_numpy(Y1).float()
        Y2_t = torch.from_numpy(Y2).float()

        gp_model = MultiTaskGaussianProcessImplicitSurfaces([X1_t, X2_t], [Y1_t, Y2_t], [T1, T2],
                                                kernel, task_kernel_params, c=50, sigma=sigma, z_limit=0.17)

        gp_model.learning(max_iter=max_iter, lr=lr)

        mean, var = gp_model.predict(XX, T2)
        estimated_surface, var = get_object_position(X, Y, Z, mean, var)
        var_list.append(np.mean(var))
        simirality_list.append(gp_model.task_params_to_psd().numpy())

        ########################################## Plot ###################################################################
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(X_body[:,0], X_body[:,1], X_body[:,2], s=200, color="navajowhite", alpha=1)

        plot_estimated_surface(estimated_surface, var)
        plot_path(position_list, i)

        # plt.savefig('../data/body/mtgpis/movie/step{}.png'.format(movie_num))
        movie_num += 1
        plt.draw()
        plt.pause(0.0001)
        plt.clf()
        ########################################## Plot ###################################################################
 


        _, direrction = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4), T2)
        direrction = direrction.numpy().T[0]
        print "direction:", direrction
    
        
        ur.send_speed_accel(alpha * direrction, t=1)

        normal, _ = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4), T2)
        normal = normal.numpy().T[0]
        print "normal   :", normal

        interval = 200
        dt = normal / interval
        print "force:", ur.force

        if ur.force < 500:
            while True:
                if ur.force > 100:
                    X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
                    Y2 = np.append(Y2, [0])[:,None]    
                    break        
                ur.send_speed_accel(-dt, t=0.1)
                # rospy.sleep(0.05)

        elif ur.force > 500:
            X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
            Y2 = np.append(Y2, [0])[:,None]
            while True:
                if ur.force < 500:
                    break
                ur.send_speed_accel(dt, t=0.1)
                # rospy.sleep(0.05)

        normal, _ = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T + 10e-4), T2)
        normal = normal.numpy().T[0]
        X2 = np.append(X2, (ur.get_current_position() + normal * 0.02)[:,None].T, axis=0)
        Y2 = np.append(Y2, [1])[:,None]    

        position_list = np.append(position_list, [ur.get_current_position()], axis = 0)


    mean, var = gp_model.predict(XX, T2)
    estimated_surface, var = get_object_position(X, Y, Z, mean, var)
    var_list.append(np.mean(var))
    simirality_list.append(gp_model.task_params_to_psd().numpy())


    ########################################## Plot ###################################################################
    # ax = fig.add_subplot(111, projection='3d')
    # plot_estimated_surface(estimated_surface, var)
    # plot_path(position_list)

    # plt.show()
    ########################################## Plot ###################################################################

    var_list = np.array(var_list)
    print("var_list:", var_list)
    print("simirality:", simirality_list)

    # np.save('../data/body/mtgpis/value/var', var_list)
    # np.save('../data/body/mtgpis/value/simirality', simirality_list)

    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)
    print "-----THE END-----"
