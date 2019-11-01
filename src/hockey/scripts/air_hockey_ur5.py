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
# from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int16

class UR5:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander() #ロボット全体に対するインタフェース
        self.manipulator = moveit_commander.MoveGroupCommander('manipulator') #MoveGroupCommanderは特定のグループのための単純なコマンドの実行を行うクラス

        self.manipulator.set_max_acceleration_scaling_factor(1)
        self.manipulator.set_max_velocity_scaling_factor(1)

        self.pad_sub = rospy.Subscriber('/hockey_2/predict/place', Int16, self.callback)

        self.pose = self.manipulator.get_current_pose() #現在のロボットの姿勢を取得
        print "########"
        print self.pose
        print "############"
        self.pad = 0

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
        # print type(self.pad)


    def main(self):

        self.start_pose = geometry_msgs.msg.Pose()
        self.target_pose1 = geometry_msgs.msg.Pose()
        self.target_pose2 = geometry_msgs.msg.Pose()
        self.target_pose3 = geometry_msgs.msg.Pose()
        self.target_pose4 = geometry_msgs.msg.Pose()
        self.target_pose5 = geometry_msgs.msg.Pose()
        self.target_pose6 = geometry_msgs.msg.Pose()
        
        self.start_pose.position.x =  0.24149264033
        self.start_pose.position.y = 0.221169067786
        self.start_pose.position.z = 0.0855258019136
        self.start_pose.orientation.x = -0.635872612116
        self.start_pose.orientation.y = 0.319763184181
        self.start_pose.orientation.z = 0.636894127345
        self.start_pose.orientation.w = 0.296282631549

        self.move_func(self.start_pose)


        '''plan1_go'''
        self.target_pose1.position.x = 0.291460000894
        self.target_pose1.position.y = 0.271686313284
        self.target_pose1.position.z = 0.0842177506964
        self.target_pose1.orientation.x = -0.635872612116
        self.target_pose1.orientation.y = 0.319763184181
        self.target_pose1.orientation.z = 0.636894127345
        self.target_pose1.orientation.w = 0.296282631549

        plan1_go = self.plan_fanc(self.target_pose1)

        print "-----plan1_go success-----"



        '''plan2_go'''
        self.target_pose2.position.x = 0.199983357219
        self.target_pose2.position.y = 0.191652326141
        self.target_pose2.position.z =0.0847234791384
        self.target_pose2.orientation.x = -0.635872612116
        self.target_pose2.orientation.y = 0.319763184181
        self.target_pose2.orientation.z = 0.636894127345
        self.target_pose2.orientation.w = 0.296282631549
        plan2_go = self.plan_fanc(self.target_pose2)

        print "-----plan2_go success-----"


        '''plan3_go'''
        # self.target_pose3.position.x = 0.141279224513
        # self.target_pose3.position.y = 0.315904815025
        # self.target_pose3.position.z = 0.0854494865821
        self.target_pose3.position.x = 0.169849446448
        self.target_pose3.position.y = 0.297044016245
        self.target_pose3.position.z = 0.0854494865821
        self.target_pose3.orientation.x = -0.635872612116
        self.target_pose3.orientation.y = 0.319763184181
        self.target_pose3.orientation.z = 0.636894127345
        self.target_pose3.orientation.w = 0.296282631549
        plan3_go = self.plan_fanc(self.target_pose3)
      
        print "-----plan3_go success-----"


        '''plan4_go'''
        self.target_pose4.position.x = 0.323826677251
        self.target_pose4.position.y =  0.135477484292
        self.target_pose4.position.z = 0.0862158405943
        self.target_pose4.orientation.x = -0.635872612116
        self.target_pose4.orientation.y = 0.319763184181
        self.target_pose4.orientation.z = 0.636894127345
        self.target_pose4.orientation.w = 0.296282631549


        plan4_go = self.plan_fanc(self.target_pose4)

        print "-----plan4_go success-----"

        '''plan5_go'''
        self.target_pose5.position.x = 0.208963633868
        self.target_pose5.position.y = 0.336352961698
        self.target_pose5.position.z = 0.0847523902081
        self.target_pose5.orientation.x = -0.635872612116
        self.target_pose5.orientation.y = 0.319763184181
        self.target_pose5.orientation.z = 0.636894127345
        self.target_pose5.orientation.w = 0.296282631549

        plan5_go = self.plan_fanc(self.target_pose5)

        print "-----plan5_go success-----"


        '''plan6_go'''
        self.target_pose6.position.x = 0.343010634698
        self.target_pose6.position.y = 0.2015733896634
        self.target_pose6.position.z = 0.0849988427976
        self.target_pose6.orientation.x = -0.635872612116
        self.target_pose6.orientation.y = 0.319763184181
        self.target_pose6.orientation.z = 0.636894127345
        self.target_pose6.orientation.w = 0.296282631549


        plan6_go = self.plan_fanc(self.target_pose6)
        rospy.sleep(1)

        print "-----plan6_go success-----"



        '''plan1_back'''
        self.execute_func(plan1_go)
        rospy.sleep(0.1)
        plan1_back = self.plan_fanc(self.start_pose)
        print "-----plan1_back success-----"

        self.execute_func(plan1_back)
        rospy.sleep(0.1)

        self.move_func(self.start_pose)


        '''plan2_back'''
        self.execute_func(plan2_go)
        rospy.sleep(0.1)
        plan2_back = self.plan_fanc(self.start_pose)
        print "-----plan2_back success-----"

        self.execute_func(plan2_back)
        rospy.sleep(0.1)
        

        '''plan3_back'''
        self.execute_func(plan3_go)
        rospy.sleep(0.1)
        print "-----plan3_back success-----"
        plan3_back = self.plan_fanc(self.start_pose)

        self.execute_func(plan3_back)
        rospy.sleep(0.1)

        self.move_func(self.start_pose)


        '''plan4_back'''
        self.execute_func(plan4_go)
        rospy.sleep(0.1)
        plan4_back = self.plan_fanc(self.start_pose)
        print "-----plan4_back success-----"

        self.execute_func(plan4_back)       
        rospy.sleep(0.1)

        self.move_func(self.start_pose)


        '''plan5_back'''        
        self.execute_func(plan5_go)
        rospy.sleep(0.1)

        plan5_back = self.plan_fanc(self.start_pose)
        print "-----plan5_back success-----"

        self.execute_func(plan5_back)
        rospy.sleep(0.1)

        self.move_func(self.start_pose)


        '''plan6_back'''
        self.execute_func(plan6_go)
        rospy.sleep(0.1)
        plan6_back = self.plan_fanc(self.start_pose)
        print "-----plan6_back success-----"

        self.move_func(self.start_pose)



        print "-----plan success-----"

        while True:
            print self.pad
            if self.pad == 8:
                self.execute_func(plan1_go)
                self.execute_func(plan1_back)
                self.move_func(self.start_pose)


            # if self.pad[1] == 1:
            #     self.execute_func(plan2_go)
            #     self.execute_func(plan2_back)


            if self.pad == 4:
                self.execute_func(plan3_go)
                self.execute_func(plan3_back)
                self.move_func(self.start_pose)


            if self.pad == 6:
                self.execute_func(plan4_go)
                self.execute_func(plan4_back) 
                self.move_func(self.start_pose)


            if self.pad == 7:
                self.execute_func(plan5_go)
                self.execute_func(plan5_back)                              
                self.move_func(self.start_pose)


            if self.pad == 9:
                self.execute_func(plan6_go)
                self.execute_func(plan6_back)
                self.move_func(self.start_pose)

            else:
                self.execute_func(plan2_go)
                self.execute_func(plan2_back)
                self.move_func(self.start_pose)

            rospy.sleep(0.1)


        # pose_list = [self.target_pose, self.start_pose]
        # t = len(pose_list)
        # loop = 1
        # print pose_list
        # while True:
        #     # print self.pad
        #     if self.pad == 8:
        #         self.target_pose.position.x = 0.321024906239
        #         self.target_pose.position.y = 0.297401590928
        #         for i in range(loop * t):
        #             self.move_func(pose_list[i % t])

        # #     if self.pad[1] == 1:
        # #         self.target_pose.position.x = 0.333680251077
        # #         self.target_pose.position.y = 0.114274167331

        # #         for i in range(loop * t):
        # #             self.move_func(pose_list[i % t])

        #     if self.pad == 4:
        #         self.target_pose.position.x = 0.163447931667
        #         self.target_pose.position.y = 0.39851998658


        #         for i in range(loop * t):
        #             self.move_func(pose_list[i % t])

        #     if self.pad == 6:
        #         self.target_pose.position.x = 0.431983800316
        #         self.target_pose.position.y = 0.173958619274
  

        #         for i in range(loop * t):
        #             self.move_func(pose_list[i % t])

        #     if self.pad == 7:
        #         self.target_pose.position.x = 0.297215569928
        #         self.target_pose.position.y = 0.429101070894

        #         for i in range(loop * t):
        #             self.move_func(pose_list[i % t])

        #     if self.pad == 9:
        #         self.target_pose.position.x = 0.468203307405
        #         self.target_pose.position.y = 0.265986965707
    
        #         for i in range(loop * t):
        #             self.move_func(pose_list[i % t])

            # rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('ur5_ik_velo', anonymous=True, disable_signals=True)
    ur5 = UR5()
    ur5.main()

