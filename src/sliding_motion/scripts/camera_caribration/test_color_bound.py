import rospy

import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

#frame = np.zeros((300,512,3), np.uint8)
view_img = np.zeros((300,512,3), np.uint8)

def nothing(x):
    pass


class ReceiveData(object):
    def __init__(self,subscribe_name):
        self.bridge = CvBridge()
        self.ImageSub = rospy.Subscriber(subscribe_name, Image, self.ImageCallback, queue_size=1)
    
    def ConvertImage(self,ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e


    def ImageCallback(self,data):
        self.frame = self.ConvertImage(data).copy()
    
# Create a black image, a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar("H_lower", 'image', 0, 255, nothing)#self.ChangeBound)
cv2.createTrackbar("H_upper", 'image', 0, 255, nothing)#self.ChangeBound)
cv2.createTrackbar("S_lower", 'image', 0, 255, nothing)#self.ChangeBound)
cv2.createTrackbar("S_upper", 'image', 0, 255, nothing)#self.ChangeBound)
cv2.createTrackbar("V_lower", 'image', 0, 255, nothing)#self.ChangeBound)
cv2.createTrackbar("V_upper", 'image', 0, 255, nothing)#self.ChangeBound)

color_range_list = []
# R
color_range_list.append([np.array([156,43,46], np.uint8), np.array([180,255,255], np.uint8)])
color_range_list.append([np.array([35,43,46], np.uint8), np.array([77,255,255], np.uint8)])
color_range_list.append([np.array([78,43,46], np.uint8), np.array([130,255,255], np.uint8)])   


node_name = "camera_extaraction_test"
try:
    rospy.init_node(node_name)
except:
    pass

rd = ReceiveData("/kinect2/sd/image_color_rect")
cv2.waitKey(100)
r = rospy.Rate(15)
while not rospy.is_shutdown():
    cv2.imshow('image',view_img)
    r.sleep()
    k = cv2.waitKey(15) & 0xFF
    if k == 27:
        break

    color_range_list[0][0][0] = cv2.getTrackbarPos("H_lower", "image")
    color_range_list[0][0][1] = cv2.getTrackbarPos("S_lower", "image")
    color_range_list[0][0][2] = cv2.getTrackbarPos("V_lower", "image")
    
    color_range_list[0][1][0] = cv2.getTrackbarPos("H_upper", "image")
    color_range_list[0][1][1] = cv2.getTrackbarPos("S_upper", "image")
    color_range_list[0][1][2] = cv2.getTrackbarPos("V_upper", "image")
    
    hsv_image = cv2.cvtColor(rd.frame, cv2.COLOR_BGR2HSV)#cv2.COLOR_BGR2LAB

    img_mask = cv2.inRange(hsv_image,color_range_list[0][0], color_range_list[0][1])
    view_img = cv2.bitwise_and(rd.frame, rd.frame, mask=img_mask)

cv2.destroyAllWindows()
