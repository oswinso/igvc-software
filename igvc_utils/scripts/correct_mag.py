#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField

mag_pub = None

def mag_callback(data):
    test = MagneticField()
    # x = y, y = -x
    # print(dir(data.magnetic_field))
    test.magnetic_field.x = -data.magnetic_field.y
    test.magnetic_field.y = data.magnetic_field.x
    test.magnetic_field.z = data.magnetic_field.z
    test.header = data.header
    mag_pub.publish(test)


def repub():
    rospy.init_node('repub')
    global mag_pub

    # republish imu covariances
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rospy.Subscriber('imu/magnetic_field', MagneticField, mag_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        repub()
    except rospy.ROSInterruptException:
        pass
