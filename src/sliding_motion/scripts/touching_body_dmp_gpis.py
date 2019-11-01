#!/usr/bin/env python
# Not generate pyc file
import sys
sys.dont_write_bytecode = True

import moveit_commander
import rospy

import numpy as np
import math

from std_msgs.msg import Int32, ColorRGBA
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg    
import moveit_msgs.msg

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import Marker

from gpis import GaussianProcessImplicitSurface
from dmp_discrete import DMPs_discrete
from calculate_hesse import eigenvalue_of_hesse_matrix

from Quaternion import Quat

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



class MoveUR5:   

    def __init__(self):
        self.id_list = 0

        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1) 

        rospy.Subscriber("/optoforce_node/optoforce_3D", geometry_msgs.msg.Vector3, self.optoforce_callback)
        rospy.Subscriber("output", PointCloud2, self.touch_points_callback)

        self.manipulator = moveit_commander.MoveGroupCommander('manipulator')

        self.manipulator.set_max_acceleration_scaling_factor(0.05)
        self.manipulator.set_max_velocity_scaling_factor(0.05)

        print "========== Printing robot position =========="
        print self.manipulator.get_current_pose()
        print "============================================="

        rospy.sleep(1)

    def touch_points_callback(self, data): 
        self.point_data = data

    def optoforce_callback(self, data):
        f = pow(data.x, 2) + pow(data.y, 2) + pow(data.z, 2)
        self.force_data = math.sqrt(f)

    def touch_point_transform(self):    

        x_points = []
        y_points = [] 
        z_points = []

        for p in pc2.read_points(self.point_data, field_names = ("x", "y", "z"), skip_nans=True):
            x_points.append(p[0])
            y_points.append(p[1])
            z_points.append(p[2])

        print "<<<<<working>>>>>"

        self.target_x_points = []
        self.target_y_points = []
        self.target_z_points = []
        self.target_points   = []

        for i in range(0,len(x_points)):

            if z_points[i] > 0.11:     
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

    def move_point(self, tar_point):
        self.manipulator.set_pose_target(tar_point)
        try:
            plan1 = self.manipulator.plan()
            # self.manipulator.stop()
                    
            self.manipulator.execute(plan1)
            # self.manipulator.stop()

        except:
            print("EERRRROOORRRR")
            rospy.sleep(0.01)
    
    def pose_position_to_list_position(self,pose_position):
        return np.array([pose_position.x, pose_position.y, pose_position.z])


           
if __name__ == '__main__':

    rospy.init_node('moveit_command_sender', anonymous=True, disable_signals=True)

    ur = MoveUR5()

    # move to home
    home = geometry_msgs.msg.Pose()

    home.position.x = float(0.304874925085)
    home.position.y = float(0.19950677174)
    home.position.z = float(0.445879639034) 

    home.orientation.x = float(-0.25615267)
    home.orientation.y = float(0.65907952)
    home.orientation.z = float(0.25615267) 
    home.orientation.w = float(0.65907952)
    
    ur.move_point(home)
    rospy.sleep(0.1)

    # show body points on rviz
    ur.touch_point_transform()
    rospy.sleep(1)

    # read orbit data and generate GP data 
    orbit_position = np.load("data/circle_orbit.npy")
    X = ur.target_points
    Y = np.zeros((np.shape(X)[0], 1))

    # print orbit_position
    # print "target:", len(ur.target_points)

    # show body points on plt
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(ur.target_points[:, 0], ur.target_points[:, 1], ur.target_points[:, 2])
    # plt.show()


    # Generate DMPs 
    dmp = DMPs_discrete(dmps=3, bfs=100, dt= 0.01)
    dmp.imitate_path(y_des=np.array([orbit_position[:,0], orbit_position[:,1], orbit_position[:,2]]))

    # Generate GPIS
    gpis = GaussianProcessImplicitSurface(X, Y)

    # move to start position
    start_pose = geometry_msgs.msg.Pose()
    start_pose.orientation = home.orientation
    start_pose.position.x = orbit_position[0][0]
    start_pose.position.y = orbit_position[0][1]
    start_pose.position.z = orbit_position[0][2]

    ur.move_point(start_pose)
    rospy.sleep(0.1)


    target = start_pose

    # for i in range(dmp.timesteps):
    #     y, dy, ddy = dmp.step(tau=1)
    #     print "y:", y
    #     target.position.x = y[0]
    #     target.position.y = y[1]
    #     target.position.z = y[2] 

    #     ur.move_point(target)
    #     rospy.sleep(0.01)

    while True:
        if ur.force_data > 500:
            break
        target.position.z -= 0.001
        ur.move_point(target)

        print "force:", ur.force_data
        rospy.sleep(0.01)

    y_track = []
    dy_track = []
    dy_norm = []
    step = 1
    current_position =  ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)
    
    while (np.linalg.norm(current_position[:2] - dmp.goal[:2]) > 10e-3) or (step < 50):
        if step == 500:
            break

        current_position = ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)

        y, dy, ddy = dmp.step(tau=1, state_fb=current_position)
        
        print "force:", ur.force_data
        if ur.force_data < 500:
            # print "----- not contact -----"
            n,d = gpis.direction_func(current_position, y)
            interval = 1000
            dt = - n / interval
            print "normal:",n
            print "dt;", dt

            while True:
                if ur.force_data > 500:
                    break
                current_position += dt
                target.position.x = current_position[0] 
                target.position.y = current_position[1]
                target.position.z = current_position[2]

                ur.move_point(target)            

                print "----- move to normal direction -----"
                rospy.sleep(0.1)

        # print "----- contact -----"    

        current_position = ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position)
        y_track.append(current_position)

        input_data_to_gp = X[np.where((X[:, 0] > current_position[0]-0.03)\
                                    & (X[:, 0] < current_position[0]+0.03)\
                                    & (X[:, 1] > current_position[1]-0.03)\
                                    & (X[:, 1] < current_position[1]+0.03))]

        eigenvalue = eigenvalue_of_hesse_matrix(input_data_to_gp)
        print "input_data_to_gp:", len(input_data_to_gp)
        print "eigenvalue:", eigenvalue

        n,d = gpis.direction_func(current_position, y)

        # n,d = gpis.direction_func(current_position, y, data_number=len(input_data_to_gp))

        if eigenvalue[0] > 0 or eigenvalue[1] > 0:
            d = 0.1 * n + d
            d /= np.linalg.norm(d)
        
        direction = 0.007 * np.linalg.norm(dy) * d

        # print "direction:", direction
        print "dy:",np.linalg.norm(dy)

        # rospy.sleep(0.1)

        next_position = current_position + direction

        target.position.x = next_position[0]
        target.position.y = next_position[1]
        target.position.z = next_position[2]

        ur.move_point(target)
        rospy.sleep(0.1)

        dy_track.append(np.copy(dy))
        y_track.append(ur.pose_position_to_list_position(ur.manipulator.get_current_pose().pose.position))
        dy_norm.append(np.linalg.norm(dy))

        step += 1

    y_track = np.array(y_track)
    dy_track = np.array(dy_track)
    dy_norm = np.array(dy_norm)
    
    ur.move_point(home)
    rospy.sleep(0.1)
    print "-----THE END-----"
