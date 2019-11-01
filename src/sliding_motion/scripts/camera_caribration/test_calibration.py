import sys
sys.dont_write_bytecode = True
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import rospy
import moveit_commander
from SubscribeImage import *
from SubscribeDepthImage import *

def touch_points_callback(data): 
    point_data = data

    print "data:", pc2.read_points(point_data, field_names = ("x", "y", "z"), skip_nans=True).data


def main():
    try:
        rospy.init_node('sample_point')
    except:
        pass
    
    rospy.Subscriber("output", PointCloud2, touch_points_callback)
        
    botharms = moveit_commander.MoveGroupCommander("manipulator")
    
    sub_img = SubscribeImage("/kinect2/sd/image_color_rect",False)
    sub_depth_img = SubscribeDepthImage("/kinect2/sd/image_depth_rect", "/kinect2/sd/camera_info", False)

    print "start"

    camera_point = np.load('camera_point.npy').T
    robot_point = np.load('robot_point.npy').T
    inv_cam =  np.linalg.pinv(camera_point)
    T = np.dot(robot_point,inv_cam)
    clip_size = [0,540,0,670]

    gp = [190,310]
    grasp = sub_depth_img.get_camera_mean_point(gp[0]+clip_size[2], gp[1]+clip_size[0])
    grasp = np.dot(T, np.array([grasp[0],grasp[1],grasp[2],1.]))[:3]

    print grasp

# def main():
#     # img = cv2.imread('./output.png')
#     try:
#         rospy.init_node('dual_grasp')
#     except:
#         pass
    
#     cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    
#     botharms = moveit_commander.MoveGroupCommander("botharms")
    
#     base_tabel_hight = -0.16+0.03
#     wait_postion = [[0.3254875521733051, 0.38, base_tabel_hight+0.15],[0.3254875521733051, -0.38, base_tabel_hight+0.15]]
        
#     clip_size = [40,370,50,630]
    
#     camera_point = np.load('camera_point.npy').T
#     robot_point = np.load('robot_point.npy').T
#     inv_cam =  np.linalg.pinv(camera_point)
#     T = np.dot(robot_point,inv_cam)
    
#     sub_img = SubscribeImage("/camera/rgb/image_rect_color",False)
#     sub_depth_img = SubscribeDepthImage("/camera/depth_registered/sw_registered/image_rect", "/camera/depth_registered/sw_registered/camera_info", False)

#     cgp = CalcGraspPoint([3,3], clip_size)#[3,5]
#     fm = FoldingMotion(wait_postion)
#     fm.set_move_group_commander(botharms)

#     rospy.sleep(0.1)
#     counter = [3,1]#np.random.randint(1, 6, (2,))#[1,5]#5#1#5#1#3#6
#     #while not rospy.is_shutdown():
#     for c in counter:
#         gp0, gp1, view_img, gp_mode, folding_direction, folding_line = cgp.calc_grasp_point(sub_img.frame, c)
#         print "fold id: "+str(c)
#         print folding_line
#         line = sub_depth_img.get_camera_mean_point(folding_line[0]+clip_size[2], folding_line[1]+clip_size[0])
#         line = np.dot(T, np.array([line[0],line[1],line[2],1.]))
#         print line
#         if folding_direction < 2:
#             line = line[0]
#         elif folding_direction < 4:
#             line = line[1]
#         print line, folding_direction
        
#         cv2.imshow('image', view_img)
#         k = cv2.waitKey(0)
#         if k == 27:
#             print "end"
#             cv2.destroyAllWindows()
#             return False
        
#         if gp_mode == 0:
#             grasp0 = sub_depth_img.get_camera_mean_point(gp0[0]+clip_size[2], gp0[1]+clip_size[0])
#             grasp1 = sub_depth_img.get_camera_mean_point(gp1[0]+clip_size[2], gp1[1]+clip_size[0])
            
#             grasp0 = np.dot(T, np.array([grasp0[0],grasp0[1],grasp0[2],1.]))[:3]
#             grasp1 = np.dot(T, np.array([grasp1[0],grasp1[1],grasp1[2],1.]))[:3]
            
#             print grasp0
            
#             pose_list = fm.run_folding_dual_arm([grasp0, grasp1], folding_direction, line)
#         else:
#             grasp0 = sub_depth_img.get_camera_mean_point(gp0[0]+clip_size[2], gp0[1]+clip_size[0])
            
#             grasp0 = np.dot(T, np.array([grasp0[0],grasp0[1],grasp0[2],1.]))[:3]
            
#             pose_list = fm.run_folding_single_arm(grasp0, folding_direction, line)
    
#     cv2.destroyAllWindows()







if __name__ == '__main__': 
    try:
        main()
    except rospy.ROSInterruptException:
        pass