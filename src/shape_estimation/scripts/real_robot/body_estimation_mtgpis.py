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

import time

# hyper parameter
alpha        = 0.01
kernel_param = 0.15
sigma        = torch.tensor(-2.5)
z_limit      = 0.17
max_iter     = 100
lr           = 0.0001

plot = True
# plot = False
save_data = True
# save_data = False
save_movie = True
# save_movie = False
object = 'low'

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
        force_x = data.x + 20
        force_y = data.y - 5
        force_z = data.z - 380

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

            if z_points[i] > 0:     # 視覚データの制限
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
        return np.array([pose_position.x, pose_position.y, pose_position.z + 0.007])

    # socketで送信すると速度のx,yの向きが逆になるのでマイナスを付ける
    def send_speed_accel(self, speed, acceleration=0.2, t=0.5):
        cmd = 'speedl([{}, {}, {}, 0, 0, 0], a={}, t={})\n'.format(-speed[0], -speed[1], speed[2], acceleration, t)
        ur.socket.send(cmd)
        rospy.sleep(t)

    def move_func(self, pose):
        self.manipulator.set_pose_target(pose)
        plan = self.manipulator.plan()
        self.manipulator.execute(plan)

        rospy.sleep(0.1)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()

    def move_orientation(self, normal):
        x = normal[0]
        y = normal[1]
        z = normal[2]

        current_pose = ur.manipulator.get_current_pose().pose


        if (x + y) > 0:
            if z < 0.65:
                current_pose.orientation.x = -0.19952163943
                current_pose.orientation.y = 0.847551668719
                current_pose.orientation.z = -0.195135590608
                current_pose.orientation.w = 0.451408224924
            
            elif 0.65 < z < 0.8:
                current_pose.orientation.x = -0.116966959016
                current_pose.orientation.y = 0.803788202612
                current_pose.orientation.z = -0.113972001957
                current_pose.orientation.w = 0.572060869673

            elif 0.8 < z < 0.95:
                current_pose.orientation.x = -0.0565693727359
                current_pose.orientation.y = 0.758331626426
                current_pose.orientation.z = -0.0546196504242
                current_pose.orientation.w = 0.647108757643
        
            else:
                current_pose.orientation.x = -0.000854411884747
                current_pose.orientation.y = 0.707179915463
                current_pose.orientation.z = -7.25989428852e-05
                current_pose.orientation.w = 0.707033119362

        else:
            if z < 0.65:
                current_pose.orientation.x = 0.17602810049
                current_pose.orientation.y = 0.484317963505
                current_pose.orientation.z = 0.173223592901
                current_pose.orientation.w = 0.83931150649


            elif 0.65 < z < 0.8:
                current_pose.orientation.x = 0.141815688697
                current_pose.orientation.y = 0.535706291899
                current_pose.orientation.z = 0.139319713193
                current_pose.orientation.w = 0.820668688799

            elif 0.8 < z < 0.95:
                current_pose.orientation.x = 0.0928812431735
                current_pose.orientation.y = 0.601356095559
                current_pose.orientation.z = 0.0911872056538
                current_pose.orientation.w = 0.788307563407
        
            else:
                current_pose.orientation.x = -0.000854411884747
                current_pose.orientation.y = 0.707179915463
                current_pose.orientation.z = -7.25989428852e-05
                current_pose.orientation.w = 0.707033119362           

        self.move_func(current_pose)


def get_object_position(x,y,z, mean, var):
    N     = len(x)
    mean  = mean.reshape(x.shape)
    var   = var.reshape(x.shape)
    mean0 = np.argmin(np.abs(mean), axis = 2)

    mean0_x, mean0_y, mean0_z, var0, var_ave = [], [], [], [], []
    for i in range(N):
        for j in range(N):
            mean0_x.append(x[i][j][mean0[i][j]])
            mean0_y.append(y[i][j][mean0[i][j]])
            mean0_z.append(z[i][j][mean0[i][j]])
            var0.append(var[i][j][mean0[i][j]])
            if z[i][j][mean0[i][j]] > 0.13:
                var_ave.append(var[i][j][mean0[i][j]])

    mean0_x = np.array(mean0_x).reshape((N, N))
    mean0_y = np.array(mean0_y).reshape((N, N))
    mean0_z = np.array(mean0_z).reshape((N, N))
    var0    = np.array(var0).reshape((N, N))
    var_ave = np.mean(np.array(var_ave))

    return [mean0_x, mean0_y, mean0_z], var0, var_ave

def plot_estimated_surface(position, var):
    N = var

    # ax.scatter(position[:,0], position[:,1], position[:,2], s=1000, c=N, cmap=cm.rainbow, linewidth=5);
    colors = cm.rainbow(N)
    Z = position[2]
    colors[Z < 0.13] = (0, 0, 0, 0)

    ax.plot_surface(position[0], position[1], position[2], facecolors=colors,linewidth=0, rstride=1, cstride=1,
    antialiased=False, shade=False,vmin=N.min(), vmax=N.max())

    m = cm.ScalarMappable(cmap=cm.rainbow)
    m.set_array(N)
    # plt.colorbar(m)

def plot_path(position, i=None):

    plt.gca().patch.set_facecolor('white')
    ax._axis3don = False

    ax.view_init(90,-45)
    # if i is None:
    #     ax.view_init(20, 60)
    # else:
    #     ax.view_init(20, (i%360)*5)
    # ax.set_xlim(0, 0.5)
    # ax.set_ylim(0, 0.5)
    # ax.set_zlim(0.05, 0.55)

    ax.set_xlim3d(0.2, 0.5)
    ax.set_ylim3d(0.2, 0.5)
    ax.set_zlim3d(0, 0.3)

    start_po = position[0]
    current_po = position[-1]

    ax.plot(position[:,0], position[:,1], position[:,2],  "o-", color="black", ms=10, mew=5, linewidth=10)
    ax.plot([current_po[0]], [current_po[1]], [current_po[2]],  "o-", color="#00aa00", ms=25, mew=5)
    ax.plot([start_po[0]], [start_po[1]], [start_po[2]],  "o-", color="red", ms=25, mew=5)

def make_test_data():

    r = np.radians(45)
    c = np.cos(r)
    s = np.sin(r)
    Rz = np.matrix([[c,-s,0],
                    [s,c,0],
                    [0,0,1]])
    
    N = 80
    x = np.linspace(0.15, 0.65, N)
    y = np.linspace(-0.25, 0.25, N)
    z = np.linspace(0.05, 0.25, N)

    X, Y, Z = np.meshgrid(x,y,z)
    for i in range(N):
        for j in range(N):
            for k in range(N):
                tmp = np.dot(Rz, np.array([X[i][j][k], Y[i][j][k], Z[i][j][k]]))
                tmp = np.array(tmp, dtype=np.float)

                X[i][j][k] = tmp[0][0]
                Y[i][j][k] = tmp[0][1]
                Z[i][j][k] = tmp[0][2]

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
    X = []
    for i in range(0,len(X1), 2):
        normal,_ = gp_model.predict_direction(X1_t[i][:, None].T)
        normal = normal.numpy().T[0]
        normal_list.append(normal)
        X.append(X1[i])
    normal_list = np.array(normal_list).reshape((-1,3))
    X = np.array(X).reshape((-1,3))

    X1 = np.concatenate([X1, X + normal_list*0.02], 0)
    Y1 = np.concatenate([Y1, np.ones(np.shape(X)[0])[:, None]], 0)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(X1[:,0],X1[:,1],X1[:,2])
    # plt.show()

    return X1, Y1


def body_error(true_surface, model):
    center_po    = np.array([0.373873986156, 0.215548561551, 0.155]) 
    true_surface = cartesian2polar(true_surface - center_po) # r, theta, phi

    N = 100
    r = np.linspace(0, 0.3, N)

    mean_list, r_list = [], []
    for i in range(len(true_surface)):
        for j in range(N):
            tmp = polar2cartesian(np.array([[r[j], true_surface[i][1], true_surface[i][2]]])) + center_po
            tmp = torch.from_numpy(tmp).float()
            mean,_ = model.predict(tmp, 1)
            mean_list.append(mean)
        
        r_list.append(r[np.abs(np.array(mean_list)).argmin()])
        mean_list = []

    return np.sqrt(np.mean((true_surface[:,0] - np.array(r_list))**2))

def head_error(true_surface, model):
    center_po    = np.array([0.223883730413, 0.372605385066, 0.11]) 
    true_surface = cartesian2polar(true_surface - center_po) # r, theta, phi

    N = 100
    r = np.linspace(0, 0.15, N)

    mean_list, r_list = [], []
    for i in range(len(true_surface)):
        for j in range(N):
            tmp = polar2cartesian(np.array([[r[j], true_surface[i][1], true_surface[i][2]]])) + center_po
            tmp = torch.from_numpy(tmp).float()
            mean,_ = model.predict(tmp, 1)
            mean_list.append(mean)
        
        r_list.append(r[np.abs(np.array(mean_list)).argmin()])
        mean_list = []

    return np.sqrt(np.mean((true_surface[:,0] - np.array(r_list))**2))

def cartesian2polar(position):
    num = position.shape[0]
    newPosition = np.empty([num,3], dtype=np.float64)
    newPosition[:,0] = np.linalg.norm(position, axis=1)
    newPosition[:,1] = np.arccos(position[:,2] / newPosition[:,0])
    newPosition[:,2] = np.arctan2(position[:,1], position[:,0])
    nan_index = np.isnan(newPosition[:,1])
    newPosition[nan_index,1] = 0
    return newPosition

def polar2cartesian(position):
    num = position.shape[0]
    newPosition = np.empty([num,3], dtype=np.float64)
    newPosition[:,0] = position[:,0] * np.sin(position[:,1]) * np.cos(position[:,2])
    newPosition[:,1] = position[:,0] * np.sin(position[:,1]) * np.sin(position[:,2])
    newPosition[:,2] = position[:,0] * np.cos(position[:,1])
    return newPosition

if __name__=="__main__":

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()
    print "force:", ur.force

    joint_home  = [-2.1334703604, -1.3442023436, -1.5670807997, -1.800930802, 1.5710361004, 1.0078269243]
    joint_start = [-2.4163258711, -1.5176289717, -1.9293277899, -1.2644713561, 1.5718277693, 0.727398634]
    # joint_start = [-2.4596217314, -1.3758891265, -2.112072293, -1.2233155409, 1.5722715855, 0.6847673655]
    # joint_start = [-2.3448455969, -1.4935887496, -2.0125439803, -1.2053983847, 1.5717079639, 0.7992721796]


    # move to home
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)

    # show body points on rviz
    ur.touch_point_transform1()
    ur.touch_point_transform2()
    rospy.sleep(1)

    # get visual data
    X1 = ur.target_points
    Y1 = np.zeros((np.shape(X1)[0], 1))
    X1, Y1 = create_dummpy_visual(X1, Y1)
    T1 = 0


    # test data
    X, Y, Z, XX = make_test_data()
    XX = torch.from_numpy(XX).float()

    ############################################# object surface ###################################################################
    X_body = np.load('../data/body/{}/mtgpis/value/body.npy'.format(object))
    body_data = np.load('../data/body/{}/mtgpis/value/true_body.npy'.format(object))
    # head_data = np.load('../data/body/{}/mtgpis/value/true_head.npy'.format(object))

    # データを前もって作り保存する、真の形状
    # X_body = ur.target_points1
    # np.save('../data/body/{}/mtgpis/value/body'.format(object), X_body)
    # surface_data = ur.target_points
    # np.save('../data/body/{}/mtgpis/value/true_surface'.format(object), surface_data)

    # true body
    # surface_data = ur.target_points
    # np.save('../data/body/{}/mtgpis/value/true_body'.format(object), surface_data)
    # true head
    # surface_data = ur.target_points
    # np.save('../data/body/{}/mtgpis/value/true_head'.format(object), surface_data)

    X_body[:,2] -= 0.035
    # X_body[:,2] -= 0.0
    body_data[:, 2] += 0.005 # 0.003
    print "-----save-----"
    ############################################# object surface ###################################################################


    # move to start position
    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_start) + "\n")
    rospy.sleep(3)

    # Show environment
    fig = plt.figure(figsize=(40, 40), dpi=40)

    position_list = np.array([ur.get_current_position()])
 
    movie_num = 1
    while True:
        print "force:", ur.force
        if ur.force > 500:
            break
        ur.send_speed_accel([0,0,-0.01],acceleration=1, t = 0.1)
        position_list = np.append(position_list, [ur.get_current_position()], axis = 0)

        ######################################### Plot ###################################################################
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(X_body[:,0], X_body[:,1], X_body[:,2], s=200, color="navajowhite", alpha=1)
        # # ax.scatter(head_data[:,0], head_data[:,1], head_data[:,2], s=200, color="navajowhite", alpha=1)
        # # ax.scatter(body_data[:,0], body_data[:,1], body_data[:,2], s=200, color="navajowhite", alpha=1)

        # # ax.scatter(X1[:,0], X1[:,1], X1[:,2], s=100, color="green")
        # plot_path(position_list)
        # if save_movie:
        #     plt.savefig('../data/body/{}/mtgpis/movie/step{}.png'.format(object, movie_num))
        #     movie_num += 1
        # plt.draw()
        # plt.pause(0.001)
        # plt.clf()
        ######################################### Plot ###################################################################
    
    rospy.sleep(1)

    # Tacticle data
    X2 = np.array([ur.get_current_position()])
    Y2 = np.array([[0]])
    X2 = np.append(X2, [ur.get_current_position() + np.array([0,0,0.02])], axis=0)
    Y2 = np.append(Y2, [1])[:,None]
    T2 = 1

    kernel = InverseMultiquadricKernelPytouch([kernel_param])


    var_list, rmse_list, simirality_list = [], [], []
    for i in range(150):
        print "========================================================================="
        print "STEP: {}".format(i)

        if i == 0:
            # task_kernel_params = torch.Tensor([[1, 10e-4], [10e-4, 1]])
            # task_kernel_params = torch.tensor([[-0.2881, -0.3729], [0.27, -1.354]]) # high
            # task_kernel_params = torch.tensor([[-0.2458, -1.8297], [0.27, 0.1005]]) # low
            # task_kernel_params = torch.tensor([[0.7650, 0.9253],[0.2700, -1.0083]]) #low body
            # task_kernel_params =torch.tensor([[-0.1025, -0.1739],[0.2700, -1.5518]]) #high body
            # task_kernel_params = torch.tensor([[-0.0213,  0.0837],[0.2700, -0.1615]])
            task_kernel_params = torch.tensor([[-0.1722, -0.2808],[0.2700,  0.1049]])



            max_iter = 200
            # max_iter=1
        else:
            task_kernel_params = gp_model.task_kernel_params
            max_iter = 50
            # max_iter=1
        print "-----------------------------------------------"
        print task_kernel_params

        X1_t = torch.from_numpy(X1).float()
        X2_t = torch.from_numpy(X2).float()
        Y1_t = torch.from_numpy(Y1).float()
        Y2_t = torch.from_numpy(Y2).float()

        gp_model = MultiTaskGaussianProcessImplicitSurfaces([X1_t, X2_t], [Y1_t, Y2_t], [T1, T2],
                                                kernel, task_kernel_params, c=100, sigma=sigma, z_limit=z_limit)

        gp_model.learning(max_iter=max_iter, lr=lr)
        print gp_model.task_kernel_params
        print gp_model.task_kernel


        # s = time.time()
    
        # body_rmse = body_error(body_data, gp_model)
        # head_rmse = head_error(head_data, gp_model)
        # rmse      = (body_rmse + head_rmse)/2
        rmse = body_error(body_data, gp_model)

        mean, var = gp_model.predict(XX, T2)
        estimated_surface, var, var_ave = get_object_position(X, Y, Z, mean, var)
        
        rmse_list.append(rmse)
        var_list.append(var_ave)
        simirality_list.append(gp_model.task_params_to_psd().numpy())
        print "rmse:", rmse
        print "var:", var_ave

        # print("time:", time.time() - s)
        ########################################## Plot ###################################################################

        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(X_body[:,0], X_body[:,1], X_body[:,2], s=200, color="navajowhite", alpha=0.8)
        # # ax.scatter(surface_data[:,0], surface_data[:,1], surface_data[:,2], s=200, color="navajowhite", alpha=1)
        # # ax.scatter(body_data[:,0], body_data[:,1], body_data[:,2], s=200, color="navajowhite", alpha=1)

        # plot_estimated_surface(estimated_surface, var)
        # plot_path(position_list, i)
        # # ax.scatter(X1[:,0], X1[:,1], X1[:,2], s=1000, color="green")

        # if save_movie:
        #     plt.savefig('../data/body/{}/mtgpis/movie/step{}.png'.format(object, movie_num))
        #     movie_num += 1

        # # plt.show()
        # plt.draw()
        # plt.pause(0.0001)
        # plt.clf()
        ########################################## Plot ###################################################################
        # print("time:", time.time() - s)

        _, direrction = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T), T2)
        direrction    = direrction.numpy().T[0]
        
        ur.send_speed_accel(alpha * direrction, t=1)

        normal, _ = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T), T2)
        normal    = normal.numpy().T[0]

        print "direction:", direrction
        print "normal   :", normal


        interval = 200
        dt = normal / interval
        if ur.force > 500:
            while True:
                if ur.force <500:
                    break
                ur.send_speed_accel(dt, t=0.1)

        # ur.move_orientation(normal)

        if ur.force < 500:
            while True:
                if ur.force > 500:
                    X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
                    Y2 = np.append(Y2, [0])[:,None]    
                    break        
                ur.send_speed_accel(-dt, t=0.1)

        elif ur.force > 500:
            while True:
                if ur.force < 500:
                    X2 = np.append(X2, ur.get_current_position()[:,None].T, axis=0)
                    Y2 = np.append(Y2, [0])[:,None]
                    break
                ur.send_speed_accel(dt, t=0.1)

        normal, _ = gp_model.predict_direction(torch.Tensor(ur.get_current_position()[:,None].T), T2)
        normal = normal.numpy().T[0]
        X2 = np.append(X2, (ur.get_current_position() + normal * 0.02)[:,None].T, axis=0)
        Y2 = np.append(Y2, [1])[:,None]    

        position_list = np.append(position_list, [ur.get_current_position()], axis = 0)



    # body_rmse = body_error(body_data, gp_model)
    # head_rmse = head_error(head_data, gp_model)
    # rmse      = (body_rmse + head_rmse)/2
    rmse = body_error(body_data, gp_model)
    mean, var = gp_model.predict(XX, T2)
    estimated_surface, var, var_ave = get_object_position(X, Y, Z, mean, var)
    
    rmse_list.append(rmse)
    var_list.append(var_ave)
    simirality_list.append(gp_model.task_params_to_psd().numpy())

    rmse_list = np.array(rmse_list)
    var_list  = np.array(var_list)
    print("rmse_list:", rmse_list)
    print("var_list:", var_list)
    print("simirality:", simirality_list)

    if save_data:
        np.save('../data/body/{}/mtgpis/value/rmse'.format(object), rmse_list)
        np.save('../data/body/{}/mtgpis/value/var'.format(object), var_list)
        np.save('../data/body/{}/mtgpis/value/simirality'.format(object), simirality_list)

    ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")
    rospy.sleep(3)
    print "-----THE END-----"
