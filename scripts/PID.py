#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class PID():
    
    def __init__(self, Kp, Ki, Kd, xtot, dt):
        self.dt = dt
        
        self.v = 0
        self.xin = 0

        self.vout = 0
    
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.xtot = xtot    
        self.integral = 0

        self.getStartTime = True

    
        self.vout = Kp*self.proportion + Ki*self.integral + Kd*self.deferential
        
    def proportional(self, v, x):
        return (self.xtot - x)
    def integrate(self, v, t):
        t = time.time()
        self.integral = self.integral + (v)*(t)
        return self.integrate
    def differentiate(self, v, t):
        differentiate = v - self.vin/t
    
    def findVout(self, vin, xin):
        vout = self.Kp*self.proportional(vin) + self.Ki*self.integrate(vin, dt) + self.Kd*self.differentiate(vin, dt)

        self.time = currentTime
        self.x = xin
        self.v = vin
        






global x, y, theta, v, omega
rospy.init_node('PID', anonymous = True)
rospy.Subscriber('/odom', Odometry, self.getPose)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        # self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

vPID = PID(K1, K2, K3, finalX, dt)
omegaPID = PID(K1, K2, K3, finalOmega, dt)

def getPose(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    v = msg.twist.twist.linear.x
    omega = msg.twist.angular.z
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    pubVel()

def pubVel():
    newV = vPID.findVout(v, x)
    newOmega = OmegaPID.findPID(omega, theta)
