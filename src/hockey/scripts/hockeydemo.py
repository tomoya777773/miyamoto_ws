# coding: utf-8


import rospy
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int16
import numpy as np
import time
import socket

class UR5_SOCKET_MOVE:
    def __init__(self):
        # self.HOST = '10.42.0.56'
        self.HOST = '192.168.1.10'
        
        self.PORT = 30001
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))
        # self.socket.send ("set_digital_out(2,True)" + "\n")


    
        # self.pad_sub = rospy.Subscriber('GamePad', Int8MultiArray, self.pad_callback)
        self.pad_sub = rospy.Subscriber('/hockey_2/predict/place', Int16, self.pad_callback)
        self.velo_sub = rospy.Subscriber('/hockey_2/velocity', Int16, self.velo_callback)

        print "###################"

        self.pad = 0
        self.velo = 0


    def pad_callback(self, msg):
        self.pad = msg.data
        print self.pad

    def velo_callback(self, msg):
        self.velo = msg.data

    def main(self):
        
        self.joint_position0 = [-2.0841243903,-1.3889802138,-2.3211498896,-1.0318577925,1.5737950802,0.1483605951]
        self.joint_position1 = [-2.1443122069,-1.6442120711,-2.0889232794,-1.0144780318,1.5757263899,0.1490092278]
        self.joint_position2 = [-2.0411618392,-1.2163680235,-2.4316452185,-1.1038463751,1.5759544373,0.1491410583]
        self.joint_position3 = [-1.6698935668,-1.4504264037,-2.280550305,-0.9975174109,1.5986046791,0.1491170973]
        # self.joint_position4 = [-2.4745441119,-1.5126870314,-2.4233880679,-0.8055680434,1.5599865913,-1.2196243445]
        self.joint_position5 = [-2.5278142134,-1.4729350249,-2.2522156874,-1.0110018889,1.5376483202,0.1491770148]
        self.joint_position6 = [-1.8780425231,-1.6146062056,-2.1206648985,-1.0002377669,1.5997322798,0.1483725756]
        self.joint_position7 = [-2.404923741,-1.6226361434,-2.1090596358,-1.0075858275,1.5417275429,0.1490571648]
        

        time.sleep(0.1)

        self.socket.send("movel({0}, a=1.0, v=1.0)".format(self.joint_position0) + "\n")

        time.sleep(2)

        # self.socket.send("movel({0}, a=0.2, v=0.1)".format(self.joint_position1) + "\n")

        # while True:
        #     print self.pad

        #     if self.pad[0] == 1:
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position1) + "\n")
        #         time.sleep(0.5)
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
        #         time.sleep(0.5)

        #     if self.pad[1] == 1:
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position2) + "\n")
        #         time.sleep(0.5)
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
        #         time.sleep(0.5)

        #     if self.pad[2] == 1:
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position3) + "\n")
        #         time.sleep(0.5)
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
        #         time.sleep(0.5)

        #     if self.pad[3] == 1:
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position4) + "\n")
        #         time.sleep(0.5)                
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
        #         time.sleep(0.5)

        #     if self.pad[4] == 1:
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position5) + "\n")
        #         time.sleep(0.5)                
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
        #         time.sleep(0.5)

        #     if self.pad[5] == 1:
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position6) + "\n")
        #         time.sleep(0.5)                
        #         self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
        #         time.sleep(0.5)

        while True:
            rospy.sleep(0.1)
            if self.velo < 20 and self.velo > 0:
                rospy.sleep(0.5)
         
                                

            #     if self.pad == 8:
            #         self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position1) + "\n")
            #         time.sleep(0.4)
            #         self.socket.send("movel({0}, a=5.0, v=1.0)".format(self.joint_position0) + "\n")
            #         time.sleep(0.4)
            

            #     if self.pad == 2:
            #         self.socket.send("movel({0}, a=2.0, v=1.0)".format(self.joint_position2) + "\n")
            #         time.sleep(0.5)
            #         self.socket.send("movel({0}, a=2.0, v=1.0)".format(self.joint_position0) + "\n")
            #         time.sleep(0.5)

            #     if self.pad == 4:
            #         self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position3) + "\n")
            #         time.sleep(0.5)
            #         self.socket.send("movel({0}, a=5.0, v=1.0)".format(self.joint_position0) + "\n")
            #         time.sleep(0.5)

            #     # if self.pad == 5:
            #     #     self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position4) + "\n")
            #     #     time.sleep(0.5)                
            #     #     self.socket.send("movel({0}, a=3.0, v=1.0)".format(self.joint_position0) + "\n")
            #     #     time.sleep(0.5)

            #     if self.pad == 6:
            #         self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position5) + "\n")
            #         time.sleep(0.5)                
            #         self.socket.send("movel({0}, a=5.0, v=1.0)".format(self.joint_position0) + "\n")
            #         time.sleep(0.5)

            #     if self.pad == 7:
            #         self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position6) + "\n")
            #         time.sleep(0.4)                
            #         self.socket.send("movel({0}, a=5.0, v=1.0)".format(self.joint_position0) + "\n")
            #         time.sleep(0.4)

            #     if self.pad == 9:
            #         self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position7) + "\n")
            #         time.sleep(0.4)                
            #         self.socket.send("movel({0}, a=5.0, v=1.0)".format(self.joint_position0) + "\n")
            #         time.sleep(0.4)
                
            #     self.pad = 0
            #     rospy.sleep(0.1)

            # else:

            if self.pad == 8:
                self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position1) + "\n")
                time.sleep(0.4)
                self.socket.send("movel({0}, a= 4.0, v=1.0)".format(self.joint_position0) + "\n")
                time.sleep(0.4)
        

            if self.pad == 2:
                self.socket.send("movel({0}, a=2.0, v=1.0)".format(self.joint_position2) + "\n")
                time.sleep(0.5)
                self.socket.send("movel({0}, a=2.0, v=1.0)".format(self.joint_position0) + "\n")
                time.sleep(0.4)

            if self.pad == 4:
                self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position3) + "\n")
                time.sleep(0.4)
                self.socket.send("movel({0}, a=6.0, v=1.0)".format(self.joint_position0) + "\n")
                time.sleep(0.4)

            # if self.pad == 5:
            #     self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position4) + "\n")
            #     time.sleep(0.5)                
            #     self.socket.send("movel({0}, a=3.0, v=1.0)".format(self.joint_position0) + "\n")
            #     time.sleep(0.5)

            if self.pad == 6:
                self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position5) + "\n")
                time.sleep(0.4)                
                self.socket.send("movel({0}, a=6.0, v=1.0)".format(self.joint_position0) + "\n")
                time.sleep(0.4)

            if self.pad == 7:
                self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position6) + "\n")
                time.sleep(0.4)                
                self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
                time.sleep(0.45)

            if self.pad == 9:
                self.socket.send("movel({0}, a=7.0, v=1.0)".format(self.joint_position7) + "\n")
                time.sleep(0.4)                
                self.socket.send("movel({0}, a=4.0, v=1.0)".format(self.joint_position0) + "\n")
                time.sleep(0.45)
            
            self.pad = 0
        rospy.sleep(0.1)




if __name__ == '__main__':
    rospy.init_node('ur5_socket_move', anonymous=True, disable_signals=True)
    ur5_socket = UR5_SOCKET_MOVE()
    ur5_socket.main()


    rospy.spin()
    ur5_socket.socket.close()