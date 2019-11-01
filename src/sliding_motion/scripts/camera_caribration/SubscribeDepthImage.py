#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy

import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from SubscribeImage import *
from copy import deepcopy

class SubscribeDepthImage(SubscribeImage):
    def __init__(self,subscribe_name, camera_info_name, display_flag=True):
        self.camera_info_sub = rospy.Subscriber(camera_info_name, CameraInfo, self.InfoCallback, queue_size=1)
        self.info_flag = False
        self.height = None
        
        while self.height == None:
            print "wait info"
            rospy.sleep(1.)

        self.tmp_image = np.empty([self.height, self.width], dtype=np.float32)
        self.world_point = np.empty([3], dtype=np.float32)
        SubscribeImage.__init__(self,subscribe_name,display_flag)
        
    def InfoCallback(self, data):
        if self.info_flag == False:
            self.f = np.array([data.P[5], data.P[0]])#[data.P[0], data.P[5]])
            self.cx = data.P[2]
            self.cy = data.P[6]
            
            self.height = data.height
            self.width = data.width
            
            self.info_flag = True
            
    def ConvertImage(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'passthrough')
            return cv_image#
        except CvBridgeError, e:
            print e
            
    def ProcessImage(self,frame):
        self.tmp_image[:] = frame[:]
        #print self.get_camera_mean_point(self.width/4, self.height/2)#self.width/2
        return frame
        
    def get_camera_point(self, u, v):
        if self.tmp_image[v,u] != np.nan:
            img_point = np.array([float(u)-self.cx, float(v)-self.cy])
            
            d = np.sqrt(img_point**2 + self.f**2)
            
            self.world_point[:2] = self.tmp_image[v,u]/d * img_point
            self.world_point[2] = np.mean(self.tmp_image[v,u]/d * self.f)
            return self.world_point[:]
        else:
            return self.tmp_image[v,u]
            
    def get_camera_mean_point(self, u, v, length = 10):
        #length = 10#5
        img_point = np.array([float(v)-self.cy, float(u)-self.cx])# np.array([float(u)-self.cx, float(v)-self.cy])
        
        d = np.sqrt(img_point**2 + self.f**2)
        #print np.nanmean(self.tmp_image[v-length:v+length+1,u-length:u+length+1])
        #print np.nanmean(self.tmp_image[v-length:v+length+1,u-length:u+length+1])/d * img_point
        #print np.nanmean(self.tmp_image[v-length:v+length+1,u-length:u+length+1])/d * self.f
        self.world_point[1:] = np.nanmean(self.tmp_image[v-length:v+length+1,u-length:u+length+1])/d * img_point
        self.world_point[0] = np.mean(np.nanmean(self.tmp_image[v-length:v+length+1,u-length:u+length+1])/d * self.f)
        self.world_point[:] = self.world_point[::-1]
        #print self.world_point[:]
        return deepcopy(self.world_point[:])

