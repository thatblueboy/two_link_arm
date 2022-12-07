#!/usr/bin/env python3

# user inputs final (x, y, z) 
# loop 
# IK find thetas, publish to controllers

from multiprocessing.dummy import JoinableQueue
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import math
# import getch

class Arm():
    def  __init__(self, x, y):
        # self.x=250+225
        # self.y=0.00
        self.x= x
        self.y= y

        self.l1=250
        self.l2=225
        self.q1=0
        self.q2=0
        self.sub=0
        print("Init")

    def algo_init(self,msg):
        self.q1=msg.position[0]
        self.q2=msg.position[1]
        self.sub.unregister()
        pass
    
    def algo(self,msg):
        try:
            print("Algorithm")
            self.q2=math.acos(((self.x*self.x)+(self.y*self.y)-(self.l1*self.l1)-(self.l2*self.l2))/(2*self.l1*self.l2))
            self.q1=(math.atan(self.y/self.x))+(math.atan((self.l2*math.sin(self.q2))/(self.l1+(self.l2*math.cos(self.q2)))))
            self.pub1.publish(-self.q1)
            self.pub2.publish((self.q2))
            print("Q1: {} Q2: {}".format(self.q1,self.q2))
            print("X: {} Y: {}".format(self.x,self.y))
            print("M1: {} M2: {}".format(-self.q1,(self.q2)))
        except:
            print("X: {} Y: {}".format(self.x,self.y))

    def main(self):
        rospy.init_node('ik_algo',anonymous=True)
        self.sub=rospy.Subscriber('/my_arm/joint_states',JointState,self.algo) #how frequently is this receiving msgs?
        self.pub1=rospy.Publisher('/my_arm/joint1_position_controller/command',Float64,queue_size=10)
        self.pub2=rospy.Publisher('/my_arm/joint2_position_controller/command',Float64,queue_size=10)
        
        rospy.spin()
        


if __name__=="__main__":
    Arm(300, 0).main()