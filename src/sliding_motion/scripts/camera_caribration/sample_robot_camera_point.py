import sys
sys.dont_write_bytecode = True
import cv2
import numpy as np

import rospy
import moveit_commander
from SubscribeImage import *
from SubscribeDepthImage import *


color_range_list = [np.array([66,118,118], np.uint8),np.array([94,255,233], np.uint8)]
kernel = np.ones((5,5),np.uint8)
erosion = 1#3
dilation = 1#2

clip_size = [0,540,0,670]

def get_camera_point(sub_img, sub_depth_img):
    img = sub_img.frame[clip_size[0]:clip_size[1],clip_size[2]:clip_size[3]]
    img_color = img[:]

    cvt_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_img = cv2.inRange(cvt_image,color_range_list[0], color_range_list[1])
    if erosion != 0:
        mask_img = cv2.erode(mask_img,kernel,iterations=erosion)
    if dilation != 0:
        mask_img = cv2.dilate(mask_img,kernel,iterations=dilation)
    
    mu = cv2.moments(mask_img, True)
    cog_x, cog_y = int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
    #print cog_x+clip_size[2], cog_y+clip_size[0]
    return sub_depth_img.get_camera_mean_point(cog_x+clip_size[2], cog_y+clip_size[0])

def main():
    # img = cv2.imread('./output.png')
    try:
        rospy.init_node('sample_point')
    except:
        pass
        
    botharms = moveit_commander.MoveGroupCommander("manipulator")
    
    sub_img = SubscribeImage("/kinect2/sd/image_color_rect",False)
    sub_depth_img = SubscribeDepthImage("/kinect2/sd/image_depth_rect", "/kinect2/sd/camera_info", False)

    rospy.sleep(0.1)
    cv2.namedWindow('image')
    diff_height = 0.25
    
    camera_point = []
    robot_point = []
    
    while not rospy.is_shutdown():        
        cv2.imshow('image',sub_img.frame[0:540,0:670])
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        elif k == ord('s'):
            cp = get_camera_point(sub_img, sub_depth_img)
            if cp[0] != np.nan:
                pose = botharms.get_current_pose().pose
                
                camera_point.append([cp[0],cp[1],cp[2],1.])
                pose.position.z += diff_height
                robot_point.append([pose.position.x, pose.position.y, pose.position.z, 1.])
                print "save point " + str(len(camera_point))
                #print "camera ", cp[0],cp[1],cp[2]
                #print "robot ",  pose.position.x, pose.position.y, pose.position.z
            else:
                print "camera ", cp[0],cp[1],cp[2]
                print "camera point nan"
        elif k == ord('o'):
            cp = get_camera_point(sub_img, sub_depth_img)
            pose = botharms.get_current_pose().pose
            
            print "camera ", cp[0],cp[1],cp[2]
            print "robot ",  pose.position.x, pose.position.y, pose.position.z

    cv2.destroyAllWindows()
    
    camera_point = np.array(camera_point)
    robot_point = np.array(robot_point)
    np.save('camera_point.npy', camera_point)
    np.save('robot_point.npy', robot_point)
    
    print camera_point, robot_point
    
    inv_cam =  np.linalg.pinv(camera_point)
    T = np.dot(robot_point,inv_cam)
    print T.shape
    print T
    
        

if __name__ == '__main__': 
    try:
        main()
    except rospy.ROSInterruptException:
        pass
