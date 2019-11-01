#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


"""circle"""

# r =  0.04757
# a = np.arange(0,361, 10)
# b = map(lambda x :math.radians(x), a)
# x = map(lambda x: 0.342 + r * math.cos(x), b)
# y = map(lambda x: 0.178 + r * math.sin(x), b)
# z = np.full(a.shape, 0.185)

# circle_list = np.array([x,y,z]).T
# print a.shape
# print  circle_list
# print(np.linalg.norm(circle_list[4] - circle_list[3]))


# fig = plt.figure()
# ax = Axes3D(fig)

# # 軸ラベルの設定
# ax.set_xlabel("X-axis")
# ax.set_ylabel("Y-axis")
# ax.set_zlabel("Z-axis")

# ax.scatter(x, y, z)

# np.save("data/circle_orbit.npy", circle_list)
# plt.show()


"""line"""

num = 50
x = np.linspace(0.365807130051, 0.160404722702, num)
y = np.linspace(0.161093867656, 0.351018413678, num)
z = np.linspace(0.187891871755, 0.203147477763, num)

line_list = np.array([x,y,z]).T

print line_list

fig = plt.figure()
ax = Axes3D(fig)

# 軸ラベルの設定
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")

ax.scatter(x, y, z)

np.save("data/line_orbit.npy", line_list)
plt.show()

    # x: 0.365807130051
    # y: 0.161093867656
    # z: 0.187891871755

    # x: 0.160404722702
    # y: 0.351018413678
    # z: 0.203147477763
