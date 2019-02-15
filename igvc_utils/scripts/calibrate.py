#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField;
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
plt.ion()
ax = fig.add_subplot(111, projection='3d')
size = int(2e8)
xs = np.empty((size,), dtype=np.float32)
ys = np.empty((size,), dtype=np.float32)
zs = np.empty((size,), dtype=np.float32)
index = 0
counter = 0


def callback(data):
    global index
    global counter
    if counter % 20 == 0:
        x = data.magnetic_field.x
        y = data.magnetic_field.y
        z = data.magnetic_field.z
        xs[index] = x
        ys[index] = y
        zs[index] = z
        index = index + 1
        counter = 0
    else:
        counter = counter + 1

def listener():
    global index
    global fig
    rospy.init_node("mag_calibrate")
    rospy.Subscriber("/imu/mag", MagneticField, callback)
    print("Started node mag_calibrate!")
    sc = ax.scatter([], [], [], s=0.2, c='#f36df9')
    
    an = np.linspace(0, 2*np.pi, 100)
    plt.plot(0.2*np.cos(an), 0.2*np.sin(an), markersize=0.01, linewidth=0.5)
    plt.xlim(-0.2,0.2)
    plt.ylim(-0.2,0.2)
    plt.axis('equal')
    while not rospy.is_shutdown():
        sc.set_offsets(np.c_[xs[0:index], ys[0:index], zs[0:index]])
        fig.canvas.draw_idle()
        plt.pause(0.0001)
    print(index)


if __name__ == "__main__":
    listener()
