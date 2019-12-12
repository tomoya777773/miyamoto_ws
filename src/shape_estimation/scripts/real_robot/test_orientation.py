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

import tf 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import numpy
import math



class MoveUR5:   

    def __init__(self):
        self.manipulator = moveit_commander.MoveGroupCommander('manipulator')
        self.manipulator.set_max_acceleration_scaling_factor(0.1)
        self.manipulator.set_max_velocity_scaling_factor(0.1)

        print "========== Printing robot position =========="
        print self.manipulator.get_current_pose()
        print "============================================="
        rospy.sleep(1)

    def move_func(self, pose):
        self.manipulator.set_pose_target(pose)
        plan = self.manipulator.plan()
        self.manipulator.execute(plan)

        rospy.sleep(0.1)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()

    def euler_to_quaternion(self, euler):
        """Convert Euler Angles to Quaternion

        euler: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion
        """
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


    def main(self):
        current_pose = self.manipulator.get_current_pose().pose
        
        self.start_pose = geometry_msgs.msg.Pose()
        
        self.start_pose.position =  current_pose.position

        normal = np.array([0.5, 0.5, -1])
        # normal = np.array([1, 0.2, 1])

        normal = normal / np.linalg.norm(normal)
        print(normal)

        # normal2 = np.array([0.1, 0.1, -0.9])
        # normal2 /= abs(normal2)
        n_x = math.atan2(normal[2], normal[1]) + 2.27
        n_y = math.atan2(normal[2], normal[0]) + 3.14
        n_z = math.atan2(normal[1], normal[0]) + 1.57 - 0.3

        print(n_x, n_y, n_z)
        print(math.degrees(n_x))
        print(math.degrees(n_y))
        print(math.degrees(n_z))
        # print quaternion_a_b(normal, normal2)
    
        alpha, beta, gamma = 0.7, 1.57, 1.45
        # alpha, beta, gamma = n_x, n_y, n_z
        alpha += 1
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
        qx = tf.transformations.quaternion_about_axis(alpha, xaxis)
        qy = tf.transformations.quaternion_about_axis(beta, yaxis)
        qz = tf.transformations.quaternion_about_axis(gamma, zaxis)
        q = tf.transformations.quaternion_multiply(qz, qy)
        q = tf.transformations.quaternion_multiply(q, qx)
        print(q)

        pose = current_pose.orientation
        print(pose)
        angles = tf.transformations.euler_from_quaternion([pose.x, pose.y, pose.z, pose.w])
        print "angle:", angles
        q = tf.transformations.quaternion_from_euler(0,math.pi/2,0)
        # aaa = tf.transformations.quaternion_from_euler(,0.2,0, 'ryxz')

        # print aaa
        self.start_pose.position = current_pose.position
        # self.start_pose.position.y = 0.221169067786
        # self.start_pose.position.z = 0.0855258019136
        self.start_pose.orientation.x = q[0]
        self.start_pose.orientation.y = q[1]
        self.start_pose.orientation.z = q[2]
        self.start_pose.orientation.w = q[3]
        print(self.start_pose)

        self.move_func(self.start_pose)

if __name__ == '__main__':
    rospy.init_node('ur5_ik_velo', anonymous=True, disable_signals=True)

    ur5 = MoveUR5()
    ur5.main()
