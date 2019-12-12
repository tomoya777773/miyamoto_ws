#! /usr/bin/env python

import rospy
from std_msgs.msg import *
from supervisor.msg import Force

import socket
import re


def robotiq_ft300():

    pub2 = rospy.Publisher('force', Force, queue_size=100)

    rospy.init_node('robotiq_ft300', anonymous=True)
    PORT = 63351  # The same port as used by the server

    rate = rospy.Rate(10)  # 10hz

    rospy.loginfo("1")

    force = Force()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while not rospy.is_shutdown():
        amount_received = 0
        amount_expected = 2
        s.connect(("163.221.44.227", PORT))
        while amount_received < amount_expected:
            data = s.recv(1024)
            amount_received += len(data)

        l = []

        for t in data.split():
            try:
                l.append(float(t))
            except ValueError:
                pass
        data = data.replace("(", "")
        data = data.replace(")", "")

        data = data.replace(",", "")

        da = data.split()

        force.x1 = float(da.pop(0))
        force.x2 = float(da.pop(0))
        force.x3 = float(da.pop(0))
        force.x4 = float(da.pop(0))
        force.x5 = float(da.pop(0))
        force.x6 = float(da.pop(0))
        pub2.publish(force)

        data = 0
        s.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rate.sleep()


if __name__ == '__main__':
    try:
        robotiq_ft300()
    except rospy.ROSInterruptException:
        pass
