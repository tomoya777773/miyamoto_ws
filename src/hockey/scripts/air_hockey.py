# coding: utf-8


"""
ロボット
・範囲
・速度
・位置
"""

import rospy
import moveit_commander
import geometry_msgs.msg
import pygame
from pygame.locals import *
import sys
import rospy
from std_msgs.msg import Int8MultiArray

class UR5:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander() #ロボット全体に対するインタフェース
        self.manipulator = moveit_commander.MoveGroupCommander('manipulator') #MoveGroupCommanderは特定のグループのための単純なコマンドの実行を行うクラス

        self.manipulator.set_max_acceleration_scaling_factor(1)
        self.manipulator.set_max_velocity_scaling_factor(1)

        self.pad_sub = rospy.Subscriber('GamePad', Int8MultiArray, self.callback)

        self.pose = self.manipulator.get_current_pose() #現在のロボットの姿勢を取得
        print "########"
        print self.pose
        print "############"

    def plan_fanc(self, pose):
        self.manipulator.set_pose_target(pose)
        plan = self.manipulator.plan()
        return plan
    
    def execute_func(self, plan):
        self.manipulator.execute(plan)

        rospy.sleep(0.1)
        # self.manipulator.stop()
        # self.manipulator.clear_pose_targets()

    def move_func(self, pose):
        self.manipulator.set_pose_target(pose)
        plan = self.manipulator.plan()
        self.manipulator.execute(plan)

        rospy.sleep(0.1)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()


    def callback(self, msg):
        self.pad = msg.data
        # print self.pad

    def main(self):

        self.start_pose = geometry_msgs.msg.Pose()
        self.target_pose1 = geometry_msgs.msg.Pose()
        # self.target_pose2 = geometry_msgs.msg.Pose()
        self.target_pose3 = geometry_msgs.msg.Pose()
        self.target_pose4 = geometry_msgs.msg.Pose()
        self.target_pose5 = geometry_msgs.msg.Pose()
        self.target_pose6 = geometry_msgs.msg.Pose()
        
        self.start_pose.position.x =  0.24149264033
        self.start_pose.position.y = 0.221169067786
        self.start_pose.position.z = 0.0875258019136
        self.start_pose.orientation.x = -0.635872612116
        self.start_pose.orientation.y = 0.319763184181
        self.start_pose.orientation.z = 0.636894127345
        self.start_pose.orientation.w = 0.296282631549

        self.move_func(self.start_pose)

        # self.target_pose.position.x =  0.24149264033
        # self.target_pose.position.y = 0.221169067786
        # self.target_pose.position.z = 0.0845258019136
        # self.target_pose.orientation.x = -0.635872612116
        # self.target_pose.orientation.y = 0.319763184181
        # self.target_pose.orientation.z = 0.636894127345
        # self.target_pose.orientation.w = 0.296282631549

        # pose_list = [self.target_pose, self.start_pose]
        # t = len(pose_list)
        # loop = 1

        '''plan1_go'''
        # self.target_pose1.position.x = 0.309960377673
        # self.target_pose1.position.y = 0.286756142876
        # self.target_pose1.position.z = 0.0860699943104

        self.target_pose1.position.x = 0.309960377673
        self.target_pose1.position.y = 0.286756142876
        self.target_pose1.position.z = 0.084699943104        
        self.target_pose1.orientation.x = -0.622985788591
        self.target_pose1.orientation.y = 0.334234637247
        self.target_pose1.orientation.z = 0.62445861868
        self.target_pose1.orientation.w = 0.332005042182

        plan1_go = self.plan_fanc(self.target_pose1)
        rospy.sleep(4)

        print "-----plan1_go success-----"


        # '''plan2_go'''
        # self.target_pose.position.x = 0.333680251077
        # self.target_pose.position.y = 0.114274167331
        # plan2_go = self.plan_fanc(self.target_pose)


        '''plan3_go'''
        self.target_pose3.position.x = 0.169849446448
        self.target_pose3.position.y = 0.297044016245
        self.target_pose3.position.z = 0.0864494865821
        self.target_pose3.orientation.x = -0.66147669003
        self.target_pose3.orientation.y = 0.223320923314
        self.target_pose3.orientation.z = 0.683634653274
        self.target_pose3.orientation.w = 0.212649981425
        
        plan3_go = self.plan_fanc(self.target_pose3)
        rospy.sleep(1)
      
        print "-----plan3_go success-----"


        '''plan4_go'''
        self.target_pose4.position.x = 0.341338412369
        self.target_pose4.position.y = 0.12248222235
        self.target_pose4.position.z = 0.0862365998287
        self.target_pose4.orientation.x = -0.568500520629
        self.target_pose4.orientation.y = 0.438364820011
        self.target_pose4.orientation.z = 0.549438604019
        self.target_pose4.orientation.w = 0.427505161413

        plan4_go = self.plan_fanc(self.target_pose4)
        rospy.sleep(1)

        print "-----plan4_go success-----"


        '''plan5_go'''
        self.target_pose5.position.x = 0.208963633868
        self.target_pose5.position.y = 0.335660118857
        self.target_pose5.position.z = 0.0860712763809
        self.target_pose5.orientation.x = -0.655471245123
        self.target_pose5.orientation.y = 0.25127346878
        self.target_pose5.orientation.z = 0.672694909741
        self.target_pose5.orientation.w = 0.233881698969

        plan5_go = self.plan_fanc(self.target_pose5)
        rospy.sleep(1)

        print "-----plan5_go success-----"

        '''plan6_go'''
        self.target_pose6.position.x = 0.343010634698
        self.target_pose6.position.y = 0.201733896634
        self.target_pose6.position.z = 0.08650017537449
        self.target_pose6.orientation.x = -0.598792160893
        self.target_pose6.orientation.y = 0.389456108065
        self.target_pose6.orientation.z = 0.583148791155
        self.target_pose6.orientation.w = 0.386922957859

        plan6_go = self.plan_fanc(self.target_pose6)
        rospy.sleep(1)

        print "-----plan6_go success-----"



        '''plan1_back'''
        self.execute_func(plan1_go)
        rospy.sleep(4)
        plan1_back = self.plan_fanc(self.start_pose)
        print "-----plan1_back success-----"
        rospy.sleep(1)

        self.execute_func(plan1_back)
        rospy.sleep(4)

        # self.execute_func(plan2_go)
        # plan2_back = self.plan_fanc(self.start_pose)
        # self.execute_func(plan2_back)
        

        '''plan3_back'''
        self.execute_func(plan3_go)
        rospy.sleep(4)
        print "-----plan3_back success-----"
        plan3_back = self.plan_fanc(self.start_pose)
        rospy.sleep(1)

        self.execute_func(plan3_back)
        rospy.sleep(4)


        '''plan4_back'''
        self.execute_func(plan4_go)
        rospy.sleep(4)
        plan4_back = self.plan_fanc(self.start_pose)
        print "-----plan4_back success-----"
        rospy.sleep(1)

        self.execute_func(plan4_back)       
        rospy.sleep(4)


        '''plan5_back'''        
        self.execute_func(plan5_go)

        plan5_back = self.plan_fanc(self.start_pose)
        rospy.sleep(4)

        print "-----plan5_back success-----"
        rospy.sleep(1)

        self.execute_func(plan5_back)
        rospy.sleep(8)


        '''plan6_back'''
        self.execute_func(plan6_go)
        plan6_back = self.plan_fanc(self.start_pose)
        rospy.sleep(1)

        print "-----plan6_back success-----"

        self.execute_func(plan6_back)      
        rospy.sleep(1)


        print "-----plan success-----"

        while True:
            print self.pad
            if self.pad[0] == 1:
                self.execute_func(plan1_go)
                self.execute_func(plan1_back)

            # if self.pad[1] == 1:
            #     self.execute_func(plan2_go)
            #     self.execute_func(plan2_back)


            if self.pad[2] == 1:
                self.execute_func(plan3_go)
                self.execute_func(plan3_back)


            if self.pad[3] == 1:
                self.execute_func(plan4_go)
                self.execute_func(plan4_back) 


            if self.pad[4] == 1:
                self.execute_func(plan5_go)
                self.execute_func(plan5_back)                              


            if self.pad[5] == 1:
                self.execute_func(plan6_go)
                self.execute_func(plan6_back)

            rospy.sleep(0.1)



        # print pose_list[]
        # while True:
            # print self.pad
            # if self.pad[0] == 1:
            #     # self.move_func1()
            #     self.target_pose.position.x = 0.409783691375
            #     self.target_pose.position.y = 0.373965080215

            #     for i in range(loop * t):
            #         self.move_func(pose_list[i % t])

            # if self.pad[1] == 1:
            #     self.target_pose.position.x = 0.333680251077
            #     self.target_pose.position.y = 0.114274167331

            #     for i in range(loop * t):
            #         self.move_func(pose_list[i % t])

            # if self.pad[2] == 1:
            #     self.target_pose.position.x = 0.163447931667
            #     self.target_pose.position.y = 0.39851998658


            #     for i in range(loop * t):
            #         self.move_func(pose_list[i % t])

            # if self.pad[3] == 1:
            #     self.target_pose.position.x = 0.431983800316
            #     self.target_pose.position.y = 0.173958619274
  

            #     for i in range(loop * t):
            #         self.move_func(pose_list[i % t])

            # if self.pad[4] == 1:
            #     self.target_pose.position.x = 0.297215569928
            #     self.target_pose.position.y = 0.429101070894

            #     for i in range(loop * t):
            #         self.move_func(pose_list[i % t])

            # if self.pad[5] == 1:
            #     self.target_pose.position.x = 0.468203307405
            #     self.target_pose.position.y = 0.265986965707
    
            #     for i in range(loop * t):
            #         self.move_func(pose_list[i % t])

            # rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('ur5_ik_velo', anonymous=True, disable_signals=True)
    ur5 = UR5()
    ur5.main()

