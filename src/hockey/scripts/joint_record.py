# coding: utf-8



import rospy
import moveit_commander
import geometry_msgs.msg
import pygame
from pygame.locals import *
import sys
from std_msgs.msg import Int16
import csv

robot = moveit_commander.RobotCommander() #ロボット全体に対するインタフェース
manipulator = moveit_commander.MoveGroupCommander('manipulator') #MoveGroupCommanderは特定のグループのための単純なコマンドの実行を行うクラス




if __name__ == '__main__':
    rospy.init_node('joint_record', anonymous=True, disable_signals=True)
    joint_record = []
    
    try:
    
        while True:
            # joint_record.append(manipulator.get_current_joint_valuea())
            joint_record.append(manipulator.get_current_joint_values())
    
    except KeyboardInterrupt:      
        print('finished')
    
    print joint_record

    f = open('joint_record1.csv', 'w')

    writer = csv.writer(f, lineterminator='\n')
    writer.writerows(joint_record)