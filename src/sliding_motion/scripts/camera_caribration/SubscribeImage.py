#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy

import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class SubscribeImage(object):
    def __init__(self,subscribe_name,display_flag=True):
        # A number of parameters to determine what gets displayed on the
        # screen. These can be overridden the appropriate launch file
        self.show_text = rospy.get_param("~show_text", True)
        self.flip_image = rospy.get_param("~flip_image", False)
    
        # Initialize a number of global variables
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.display_image = None
        
        self.resize_window_width = 0
        self.resize_window_height = 0
        
        self.display_wait_ = 30
        self.display_flag = display_flag
        
        self.max_depth = 2.
        
        # Create the main display window
        if self.display_flag:
            self.cv_window_name = self.__class__.__name__#self.node_name
            """
            cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
            if self.resize_window_height > 0 and self.resize_window_width > 0:
                cv.ResizeWindow(self.cv_window_name, self.resize_window_width, self.resize_window_height)
            """
        
        #self.display_flag = True
        
        
        # Create the cv_bridge object
        self.bridge = CvBridge()
    
        self.ImageSub = rospy.Subscriber(subscribe_name, Image, self.ImageCallback, queue_size=1)
        
    def ImageCallback(self, data):
        # Store the image header in a global variable
        self.image_header = data.header

        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.ConvertImage(data)

        # Some webcams invert the image
        if self.flip_image:
            frame = cv2.flip(frame, 0)
        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()
        
        processed_image = self.ProcessImage(frame)
        self.processed_image = processed_image.copy()
        
        if self.display_flag:
            self.display_image = self.processed_image/self.max_depth
            cv2.imshow(self.__class__.__name__, self.display_image)
            cv2.waitKey(self.display_wait_)
                   
        if self.show_text:
            self.ShowText()

    def ConvertImage(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e
    
    def ProcessImage(self,frame):
        return frame
    
    def ShowText(self):
        pass
        
