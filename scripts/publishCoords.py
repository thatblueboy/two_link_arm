#!/usr/bin/env python3

import cv2
import rospy
from two_link_arm.msg import Coords
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera.msg import Point

pub = rospy.Publisher('/coordinates', Point, queue_size = 10 )
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10)

point = Point()
point.x = 0
point.y = 0

while not rospy.is_shutdown():
    pub.publish(point)