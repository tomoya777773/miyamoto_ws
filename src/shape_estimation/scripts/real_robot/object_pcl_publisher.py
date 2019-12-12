#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import ctypes
import struct
import pcl
import ros_numpy

tmp_pcd_name = "tmp_cloud.pcd"

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]

import numpy as np



def rgb_to_hue(r,g,b):
    M = max(r,g,b)
    m = min(r,g,b)
    c = M - m  # chroma.
 
    if c == 0:
        return None  # undefined.
    elif r == M:
        h2 = (g-b)/c % 6
    elif g == M:
        h2 = (b-r)/c + 2
    else:
        h2 = (r-g)/c + 4
 
    return 60*h2

def rgb_to_hsv(r,g,b):
    hue = rgb_to_hue(r,g,b)
    c = max(r,g,b) - min(r,g,b)
    val = max(r,g,b)
    sat = 0 if c == 0 else c/val
    return np.array([hue,sat,val])
 
def hsv_to_rgb(hsv):
    """
    >>> from colorsys import hsv_to_rgb as hsv_to_rgb_single
    >>> 'r={:.0f} g={:.0f} b={:.0f}'.format(*hsv_to_rgb_single(0.60, 0.79, 239))
    'r=50 g=126 b=239'
    >>> 'r={:.0f} g={:.0f} b={:.0f}'.format(*hsv_to_rgb_single(0.25, 0.35, 200.0))
    'r=165 g=200 b=130'
    >>> np.set_printoptions(0)
    >>> hsv_to_rgb(np.array([[[0.60, 0.79, 239], [0.25, 0.35, 200.0]]]))
    array([[[  50.,  126.,  239.],
            [ 165.,  200.,  130.]]])
    >>> 'r={:.0f} g={:.0f} b={:.0f}'.format(*hsv_to_rgb_single(0.60, 0.0, 239))
    'r=239 g=239 b=239'
    >>> hsv_to_rgb(np.array([[0.60, 0.79, 239], [0.60, 0.0, 239]]))
    array([[  50.,  126.,  239.],
           [ 239.,  239.,  239.]])
    """
    input_shape = hsv.shape
    hsv = hsv.reshape(-1, 3)
    h, s, v = hsv[:, 0], hsv[:, 1], hsv[:, 2]

    i = np.int32(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i = i % 6

    rgb = np.zeros_like(hsv)
    v, t, p, q = v.reshape(-1, 1), t.reshape(-1, 1), p.reshape(-1, 1), q.reshape(-1, 1)
    rgb[i == 0] = np.hstack([v, t, p])[i == 0]
    rgb[i == 1] = np.hstack([q, v, p])[i == 1]
    rgb[i == 2] = np.hstack([p, v, t])[i == 2]
    rgb[i == 3] = np.hstack([p, q, v])[i == 3]
    rgb[i == 4] = np.hstack([t, p, v])[i == 4]
    rgb[i == 5] = np.hstack([v, p, q])[i == 5]
    rgb[s == 0.0] = np.hstack([v, v, v])[s == 0.0]

    return rgb.reshape(input_shape)


def publish_pointcloud(output_data, input_data):
    # convert pcl data format
    pc_p = np.asarray(output_data.points)
    pc_c = np.asarray(output_data.colors)
    tmp_c = np.c_[np.zeros(pc_c.shape[1])]
    tmp_c = np.floor(pc_c[:,0]) * 2**16 + np.floor(pc_c[:,1]) * 2**8 + np.floor(pc_c[:,2]) # 16bit shift, 8bit shift, 0bit shift

    pc_pc = np.c_[pc_p, tmp_c]

    # publish point cloud
    output = pc2.create_cloud(Header(frame_id=input_data.header.frame_id), FIELDS , pc_pc)
    pub = rospy.Publisher('/output', PointCloud2, queue_size=1)
    pub.publish(output)

def publish_pointcloud2(output_data, input_data):
    # convert pcl data format
    pc_p = np.asarray(output_data.points)
    pc_c = np.asarray(output_data.colors)
    tmp_c = np.c_[np.zeros(pc_c.shape[1])]
    tmp_c = np.floor(pc_c[:,0]) * 2**16 + np.floor(pc_c[:,1]) * 2**8 + np.floor(pc_c[:,2]) # 16bit shift, 8bit shift, 0bit shift

    pc_pc = np.c_[pc_p, tmp_c]

    # publish point cloud
    output = pc2.create_cloud(Header(frame_id=input_data.header.frame_id), FIELDS , pc_pc)
    pub = rospy.Publisher('/output2', PointCloud2, queue_size=1)
    pub.publish(output)

def pcd_color_data_change_rgb(color_data):
    test = color_data
    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f' ,test)
    i = struct.unpack('>l',s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000)>> 16
    g = (pack & 0x0000FF00)>> 8
    b = (pack & 0x000000FF)
    # print r,g,b # prints r,g,b values in the 0-255 range

    return r,g,b

def rotate_xyz(data):

    # px = np.pi/4*5
    # py = -np.pi/4
    # pz = np.pi/3
    px = -2.18*np.pi/3
    py = -1.5*np.pi/60
    pz = 0.9*np.pi/4
    px2= np.pi/60

    # 物体座標系の 1->2->3 軸で回転させる
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(px), -np.sin(px)],
                   [0, np.sin(px), np.cos(px)]])
    Ry = np.array([[np.cos(py), 0, np.sin(py)],
                   [0, 1, 0],
                   [-np.sin(py), 0, np.cos(py)]])
    Rz = np.array([[np.cos(pz), -np.sin(pz), 0],
                   [np.sin(pz), np.cos(pz), 0],
                   [0, 0, 1]])
    Rx2= np.array([[1, 0, 0],
                   [0, np.cos(px2), -np.sin(px2)],
                   [0, np.sin(px2), np.cos(px2)]])

    data = Rx.dot(data.T) 
    # data[2,:] += 0.5
    data = Rz.dot(Ry.dot(data))
    # data[0,:] += 0.3
    # data[1,:] += 0.02
    data = Rx2.dot(data)
    # data[0,:] += 0.82
    # data[1,:] -= 0.27
    # data[2,:] += 0.8

    data[0,:] += 0.85
    data[1,:] -= 0.24
    data[2,:] += 0.795

    return data.T

    # return np.dot(np.dot(np.dot(Ry,Rx),Rz),data.T).T

def on_new_point_cloud(data):
    pc = ros_numpy.numpify(data)

    height = pc.shape[0]
    width  = pc.shape[1]
    # print pc['rgb']
    # print rgb_to_hsv(pc['rgb'])
    p,c = [], []
    for i in range(height):
        for j in range(width):
            R,G,B = pcd_color_data_change_rgb(pc['rgb'][i][j])
            # HSV = rgb_to_hsv(R,G,B)
            # # print HSV
            # H = HSV[0]
            # S = HSV[1]
            # V = HSV[2]
            # # print 
            # if H > 100:

            # if 100 < R < 256 and 150 < G < 256 and 100 < B < 250:
            # if 0 <= H < 100 and 0 <= S < 1 and 200 <= V < 256:
            if  -0.4 < pc['x'][i][j] < 0.2 and -0.15 < pc['y'][i][j] < 0.3 and 0.7 < pc['z'][i][j] < 1.5:
                # print "----------------"
                p.append([pc['x'][i][j], pc['y'][i][j], pc['z'][i][j]])
                c.append([R, G, B])

    p = np.array(p)
    p = rotate_xyz(p)

    c = np.array(c)
    
    # print p[p[:,2] > 0]
    # tmp = (0 < p[:,0]) & (p[:,0] < 0.5) & (0 < p[:,1]) & (p[:,1] < 0.5) & (0.001 < p[:,2]) & (p[:,2] < 0.2) & (p[:,0] + p[:,1] < 0.8)
    # tmp = 0 < p[:,1] and p[:,1] < 0.5 and p[:,2] > 0
    # tmp = (0.1 < p[:,0] + p[:,1]) & (p[:,0] + p[:,1] < 0.8) & (-0.6 < p[:,1] - p[:,0]) & (p[:,1] - p[:,0] < 0.3) & (0.001 < p[:,2]) & (p[:,2] < 0.2)
    tmp1 = (0.1 < p[:,0] + p[:,1]) & (p[:,0] + p[:,1] < 0.8) & (-0.6 < p[:,1] - p[:,0]) & (p[:,1] - p[:,0] < 0) & (0.1 < p[:,2]) & (p[:,2] < 0.2)
    tmp2 = (0.1 < p[:,0] + p[:,1]) & (p[:,0] + p[:,1] < 0.8) & (-0.6 < p[:,1] - p[:,0]) & (p[:,1] - p[:,0] < 0.3) & (0.1 < p[:,2]) & (p[:,2] < 0.2)

    c1 = c[tmp1]
    p1 = p[tmp1]

    c2 = c[tmp2]
    p2 = p[tmp2]

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(p1)
    pcd1.colors = o3d.utility.Vector3dVector(c1)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(p2)
    pcd2.colors = o3d.utility.Vector3dVector(c2)

    return pcd1, pcd2

def callback(data):
    pcl1, pcl2 = on_new_point_cloud(data)
    print "-----publish pcl-----"
    publish_pointcloud(pcl2, data)

    pcl1 = pcl1.voxel_down_sample(voxel_size = 0.05)
    publish_pointcloud2(pcl1, data)


if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/kinect2/sd/points', PointCloud2, callback)
    rospy.spin()