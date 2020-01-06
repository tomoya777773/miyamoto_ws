#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import axes3d

X, Y = np.mgrid[:10:200j, :10:200j]
Z = -2 * X - 5 * Y + 20
print(Z.shape)
# カラーマップに基づき、色を生成する。
norm = plt.Normalize(vmin=Z.min(), vmax=Z.max())
colors = plt.cm.jet(norm(Z))
# Z < 0 のパッチはアルファチャンネルを0として色を透明にする。
colors[Z < 0] = (0, 0, 0, 0)

# 描画する。
fig = plt.figure(figsize=(10, 8))
ax = fig.gca(projection="3d")
ax.set_xlim(0, 10)
ax.plot_surface(X, Y, Z, facecolors=colors, rstride=1, cstride=1)
ax.set_zlim3d(bottom=0)
ax.view_init(elev=20, azim=20)
plt.show()
