#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64




class tracker():
    def __init__(self):
        print("init")

        #data
        self.q1 = 0
        self.q2 = 0
        self.Q1 = 0
        self.Q2 = 0
        self.l1 = 250
        self.l2 = 225
        self.xpoints = []
        self.ypoints = []
        

        rospy.init_node('tracker',anonymous=True)
        print("take once")

        self.q1, self.q2 = (rospy.wait_for_message('/my_arm/joint_states',JointState, timeout=None)).position
        # print()
        print("take more")
        
        while not rospy.is_shutdown():

            self.sub=rospy.Subscriber('/my_arm/joint_states',JointState,self.collectCurrentState)
            inp = input()
            if inp == "p":
                self.plot()
              

        # self.sub1 = rospy.Subscriber('/my_arm/joint1_position_controller/command', Float64, self.collectQ1)
        # self.sub2 = rospy.Subscriber('/my_arm/joint2_position_controller/command', Float64, self.collectQ2)

        # self.Q1 = rospy.wait_for_message('/my_arm/joint1_position_controller/command', Float64, timeout=None)
        # self.Q2 = rospy.wait_for_message('/my_arm/joint2_position_controller/command', Float64, timeout=None)
        
        # self.collectQ1(rospy.wait_for_message('/my_arm/joint1_position_controller/command', Float64, timeout=None))
        # self.collectQ2(rospy.wait_for_message('/my_arm/joint2_position_controller/command', Float64, timeout=None))
        
        # self.x = self.fk_x(self.q1, self.q2)
        # self.y = self.fk_y(self.q1, self.q2)
        # print(self.x)
        # print(self.y)
        # print(self.q1)
        # print(self.q2)
        # rospy.spin()

        
        # self.plot()
        
    def main(self):
        pass

    def plot(self):
        self.xpoints = np.array(self.xpoints)
        self.ypoints = np.array(self.ypoints)
        plt.plot(self.xpoints, self.ypoints)
        plt.show()
       

    def fk_x(self, q1, q2):
        return self.l1*math.cos(-q1) + self.l2*math.cos(-q2-q1)

    def fk_y(self, q1, q2):
        return self.l1*math.sin(-q1) + self.l2*math.sin(-q2-q1)

    def closeEnough(self, a, b):
        if abs(float(a) - float(b)) <= 0.01:
        # if a == b:
            return True
        else:
            return False

    def collectQ1(self, Q1):
        self.Q1 = Q1.data

    def collectQ2(self, Q2):
        self.Q2 = Q2.data

    def collectCurrentState(self, msg):
        # print("rcvd")
        # print(self.q1, self.q2)
        # print("q1q2")
       
        q1, q2 = msg.position
        # print(q1, q2)
        self.xpoints.append(self.fk_x(q1, q2))
        self.ypoints.append(self.fk_y(q1, q2))



        # if not self.closeEnough(self.q1, q1) or not self.closeEnough(self.q2, q2):
        #     self.q1, self.q2 = q1, q2
        #     print(self.q1, self.q2)
        #     print(self.fk_x(self.q1, self.q2), self.fk_y(self.q1, self.q2))
        #     self.xpoints.append(self.fk_x(self.q1, self.q2))
        #     self.ypoints.append(self.fk_y(self.q1, self.q2))



        #position: [-1.4604116696122205, 1.4719025868702973]


if __name__=="__main__":
    tracker()






