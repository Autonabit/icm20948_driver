#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import MagneticField,Imu
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import numpy as np


def update_marker(mag_message):
    mag = np.array([mag_message.magnetic_field.x, mag_message.magnetic_field.y, mag_message.magnetic_field.z])
    mag = mag / np.linalg.norm(mag)
    rospy.loginfo(mag)

    marker = Marker(
        header=Header(frame_id="imu", stamp=rospy.Time.now()),
        id=0,
        type=Marker.ARROW,
        pose=Pose(Point(0,0,0),
                      Quaternion(0, 0, 0, 1)),
        action=Marker.ADD,
        points=[Point(0,0,0), Point(mag[0],mag[1],mag[2])],
        color=ColorRGBA(1, 0, 0, 1),
        scale=Vector3(0.2,0.4,0),
        lifetime=rospy.Duration()
    )

    marker_pub.publish(marker)

if __name__=="__main__":
    rospy.init_node("mag_marker_publisher")


    marker_pub = rospy.Publisher('mag_marker', Marker, queue_size=10)
    mag_sub = rospy.Subscriber('/imu/mag', MagneticField, update_marker, queue_size=1)

    rospy.spin()