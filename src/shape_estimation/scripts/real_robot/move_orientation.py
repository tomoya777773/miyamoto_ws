#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import rospy
import numpy as np 
import time
import math 

HOST = '163.221.44.227'
PORT = 63351

def toBytes(str):
    return bytes(str.encode())


def send_speed_accel(speed, acceleration=0.1, t=1):
    PORT = 30001
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

    cmd = 'speedl([{}, {}, {}, 1, 1, 1], a={}, t={})\n'.format(-speed[0], -speed[1], speed[2], acceleration, t)
    s.send(cmd)
    rospy.sleep(t)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send(toBytes("get_actual_tcp_pose()" +"\n"))
s.send(toBytes(" get_actual_tool_flange_pose()" +"\n"))

data = s.recv(1024)
s.close()

print("Receved", repr(data))

data = data.replace("(", "")
data = data.replace(")", " , ")
data = data.replace(",", "")
l = []
for t in data.split():
    try:
        l.append(float(t))
    except ValueError:
        pass
print(l)

normal = np.array([0.0001, 0.02, 0.99])


n_x = math.atan2(normal[2], normal[1]) 
n_y = math.atan2(normal[2], normal[0]) 
n_z = math.atan2(normal[1], normal[0]) 

# n_x = math.atan2(normal[2], normal[1])
# n_y = math.atan2(normal[2], normal[0])
# n_z = math.atan2(normal[0], normal[1])

print(n_x, n_y, n_z)
print(math.degrees(n_x))
print(math.degrees(n_y))
print(math.degrees(n_z))

print("Receved", repr(data))
print("Program finish")

speed = np.array([0.0,0.0,0.0])
# send_speed_accel(speed)

# ur.socket.send("movel({0}, a=0.2, v=0.2)".format(joint_home) + "\n")



