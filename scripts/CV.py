#!/usr/bin/env python3

import cv2
import rospy
from two_link_arm.msg import Coords
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class computerVision():

    def __init__(self):
        rospy.init_node('CV', anonymous=True)
        self.bridge = CvBridge()

        # rospy.wait_for_message('two/mybot/camera/image_raw', sensor_msgs.)

        rospy.Subscriber('two/mybot/camera/image_raw', Image, self.findCoords)
        self.pub = rospy.Publisher('/centroid_coordinates', Coords, queue_size= 10)

        rospy.spin()

    def findCoords(self, img):
        img = self.bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") #converting image to cv2 format
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)    #image was in RGB but cv2 reads BGR

        print(type(img))

        # imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert img to grayscale
        imgray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) #convert img to grayscale


        ret, thresh1 = cv2.threshold(imgray, 225, 255, cv2.THRESH_BINARY) #get binary file of grayscale

        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #find contour

        cv2.drawContours(img, contours, -1, (0, 255, 0), 3) #draw contours, not required

        # # calculate moments of binary image
        M = cv2.moments(thresh1)

        # # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        print(cX, cY)

        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)

        # self.publishCoords(cX, cY)
        # cv2.imshow("image", imgray)
        cv2.imshow("Image", img)
        cv2.waitKey(3)




    # def publishCoords(self, cX, cY):
    #     # coords = [cX, cY]
    #     coords = Coords()
    #     coords.x = cX
    #     coords.y = cY
    #     self.pub.publish(coords)


if __name__ == '__main__':
    computerVision()












       
       
# img = cv2.imread('blue.png')
# cv2.imshow('Image', img)
# imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.imshow('Image GRAY', imgray)
# # ret, thresh = cv2.threshold(imgray, 127, 255, 0)
# ret, thresh1 = cv2.threshold(imgray, 20, 255, cv2.THRESH_BINARY)
# ret, thresh2 = cv2.threshold(imgray, 20, 255, cv2.THRESH_BINARY_INV)

# # cv2.imshow('thresh', thresh2)

# contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
# # print("Number of contours = " + str(len(contours)))
# # print(contours[0])

# cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
# # cv2.drawContours(imgray, contours, -1, (0, 255, 0), 3)
# cv2.imshow('Image', img)


# # cv2.waitKey(0)
# # cv2.destroyAllWindows()

# # calculate moments of binary image
# M = cv2.moments(thresh1)

# # calculate x,y coordinate of center
# cX = int(M["m10"] / M["m00"])
# cY = int(M["m01"] / M["m00"])

# # put text and highlight the center
# cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
# cv2.putText(img, "centroid ", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
# print(cX, cY)
# # print
# # display the image
# cv2.imshow("Image", img)
# cv2.waitKey(0)

