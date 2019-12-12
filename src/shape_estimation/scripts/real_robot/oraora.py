#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
        target_po = current_pose.position
        print(target_po)
        normal = np.array([0.5, 0.5, 0])

        HOST = '163.221.44.227'
        PORT = 30001
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))

        # cmd = 'movel([{}, {}, {}, {}, {}, {}], a={},v={}, t={})\n'.format(target_po.x, target_po.y, 0.5, normal[0], normal[1], normal[2], 0.0005, 0.0005, 10)
        # s.send(cmd)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('ur5_ik_velo', anonymous=True, disable_signals=True)

    ur5 = MoveUR5()
    ur5.main()

