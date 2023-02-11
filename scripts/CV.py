#!/usr/bin/env python3

import cv2
import rospy
from two_link_arm.msg import Coords
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera.msg import Point


class computerVision():

    def __init__(self):
        self.x = 0
        self.y = 0
        rospy.init_node('CV', anonymous=True)
        self.bridge = CvBridge()

        # rospy.wait_for_message('two/mybot/camera/image_raw', sensor_msgs.)
        self.sub = rospy.Subscriber('/coords', Point, self.getCoords)
        rospy.Subscriber('/cam/rgb/image_raw', Image, self.findCoords)
        self.pub = rospy.Publisher('/centroid_coordinates', Coords, queue_size= 10)

        rospy.spin()

    def findCoords(self, img):
       
        img = self.bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") 
        dimensions = img.shapek
        print(dimensions)
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)  
        imgray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        ret, thresh1 = cv2.threshold(imgray, 225, 255, cv2.THRESH_BINARY) 
        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        M = cv2.moments(thresh1)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        cx, cy = self.transform(cx, cy)

        self.findVect(cx, cy, self.x, self.y)

        # cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
        # cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        cv2.imshow("Image", img)
        cv2.waitKey(1)
    
    def findVect(self, x1, y1, x2, y2):
        dx = x1 - x2
        dy = y1 - y2
        
        point = Point()
        point.x = dx
        point.y = dy

        self.pub.publish(point)

    def transform(self, x, y):
        

        return x, y
    
    def getCoords(self, point):
        self.x = point.x
        self.y = point.y


if __name__ == '__main__':
    computerVision()
