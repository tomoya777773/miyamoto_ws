#!/usr/bin/env python
import moveit_commander
import roslib
import sys
import copy
import rospy
import numpy as np
from math import pi
import std_msgs.msg
from std_msgs.msg import Float32MultiArray
import moveit_msgs.msg
import geometry_msgs.msg
# controller
import pygame
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import keyboard
import tf

import tty
import sys
import termios



class Rviz_Frames:
    def __init__(self):

        # controller
        pygame.display.init()
        # pygame.joystick.init()
        # pygame.joystick.Joystick(0).init()
        self.orig_settings = termios.tcgetattr(sys.stdin)
        #tty.setraw(sys.stdin)
        self.x= 0.55 
        self.y= 0.535
        self.z= 0.675

        self.a =-0.105
        self.b = 1.125
        self.c =-2.425
        #
        # print("X1-Axis: ", pygame.joystick.Joystick(0).get_axis(0))
        # print("Y1-Axis: ", pygame.joystick.Joystick(0).get_axis(1))

    def get_joy_pos(self, axis):
        # X :axis = 0, Y :axis = 1
        val = pygame.joystick.Joystick(0).get_axis(axis)
        pygame.event.pump()

        if val == 0:
            return 0.0
        if val > 0:
            return 1.0
        else:
            return -1.0

    def enviorment(self):

        rospy.sleep(2)
  
    def handle_publish_tf(self):
        self.br = tf.TransformBroadcaster()

        

        self.t = geometry_msgs.msg.TransformStamped() 
        self.t.header.frame_id = "base_link" 
        self.t.header.stamp = rospy.Time.now() 
        self.t.child_frame_id = "camera_link" 
        self.t.transform.translation.x = self.x
        self.t.transform.translation.y = self.y
        self.t.transform.translation.z = self.z
        self.q = quaternion_from_euler(self.a, self.b, self.c) 
        self.t.transform.rotation.x = self.q[0] 
        self.t.transform.rotation.y = self.q[1] 
        self.t.transform.rotation.z = self.q[2] 
        self.t.transform.rotation.w = self.q[3] 
        self.br.sendTransformMessage(self.t)


        x = 0
        #tty.setraw(sys.stdin)
        #x=sys.stdin.read(1)[0]
        #print("You pressed", x)

        if(x=="1"):
            self.x += 0.005
        if(x=="2"):
            self.x -= 0.005
        if(x=="3"):
            self.y += 0.005
        if(x=="4"):
            self.y -= 0.005
        if(x=="5"):
            self.z += 0.005
        if(x=="6"):
            self.z -= 0.005
        
        if(x=="a"):
            self.a += 0.005
        if(x=="b"):
            self.a -= 0.005
        if(x=="c"):
            self.b += 0.005
        if(x=="d"):
            self.b -= 0.005
        if(x=="e"):
            self.c += 0.005
        if(x=="f"):
            self.c -= 0.005
        print "a", self.a
        print "b", self.b
        print "c", self.c
        print "x", self.x
        print "y", self.y
        print "z", self.z 
        #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)    



        """
      
        self.t = geometry_msgs.msg.TransformStamped()


        self.t.transform.translation.x= 1
        self.t.transform.translation.y = 1
        self.t.transform.translation.z = 0

        self.target_pose2 = geometry_msgs.msg.Quaternion()

        self.target_pose2.w = 1
        self.target_pose2.x = 1
        self.target_pose2.y = 1
        self.target_pose2.z = 1

        # self.tfStamp.header/frame_id = 'base'
        self.br.sendTransformMess(self.target_pose.x, self.target_pose.y, self.target_pose.z, tf.transformations.quaternion_from_euler(0, 0, 1),
                              rospy.Time.now(), "camera_link", "base_link")"""
        print("=========    Publishing ============")


if __name__ == '__main__':
    rospy.init_node('rviz_frames_publisher', anonymous=True)
    print("=========    Start to publish TF ============")

    rv = Rviz_Frames()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rv.handle_publish_tf()
        
        rate.sleep()

    # robot = moveit_commander.RobotCommander()
    # mani = moveit_commander.MoveGroupCommander('manipulator')

    # mani.set_max_acceleration_scaling_factor(0.1)
    # mani.set_max_velocity_scaling_factor(0.1)

    # while True:
    #   orie = mani.get_current_pose().pose.orientation
    #   e = list(tf.transformations.euler_from_quaternion((orie.x, orie.y, orie.z, orie.w)))
    #   print e
    #   rospy.sleep(1)
