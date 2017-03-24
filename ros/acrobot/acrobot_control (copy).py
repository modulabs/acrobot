#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan2,sqrt,fabs
from numpy import *
class Acrobot:
    m = array([1, 2])
    l = array([1, 2])
    lc = array([0.5, 1])
    Ic = array([0.083, 0.33])
    g = 9.81
    b = array([.1, .1])

    def __init__(self, m1, m2, l1, l2, lc1, lc2, Ic1, Ic2):
        self._m = array([m1, m2])
        self._l = array([l1, l1])
        self._lc = array([lc1, lc2])
        self._Ic = array([Ic1, Ic2])
    
    def manipulator_dynamics(self, q, qd):
         # keep it readable:
        m1=self.m[0]
        m2=self.m[1]
        l1=self.l[0]
        l2=self.l[1]
        g=self.g
        lc1=self.lc[0]
        lc2=self.lc[1]
        b1=self.b[0]
        b2=self.b[1]
        I1 = self.Ic[0] + self.m[0]*self.lc[0]**2
        I2 = self.Ic[1] + self.m[1]*self.lc[1]**2
        m2l1lc2 = m[1]*l[0]*lc[1]  # occurs often!

        c = cos(q[0:2,:])
        s = sin(q[0:2,:])
        s12 = sin(q[0,:]+q[1,:])

        h12 = I2 + m2l1lc2*c[1]
        H = array([[ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12],
                [h12, I2] ])

        C = array([[-2*m2l1lc2*s[1]*qd[1], -m2l1lc2*s[1]*qd[1]],
                [m2l1lc2*s[1]*qd[0], 0] ])
        G = g*array([[ m1*lc1*s[0] + m2*(l1*s[1]+lc2*s12)],
            [m2*lc2*s12] ])
            
        # accumate total C and add a damping term:
        C = C.dot(qd) + G + array([[b1],[b2]]).dot(qd)

        B = array([[0], [1]])

        return (H, C, B)

def listen_joint_state(data):
    q[0] = data.data[0]
    q[1] = data.data[1]

#Define a RRBot joint positions publisher for joint controllers.
def acrobot_control_publisher():

    Acrobot acrobot(1, 2, 1, 2, 0.5, 1, 0.083, 0.33)
    q = deg2rad(array([[45],[0]]))
    q_dot = deg2rad(array([[0],[0]]))

    des_q2_ddot = 0
    des_q2_dot = 0
    des_q2 = 0
    
    alpha = 22
    kp = 50
    kd = 50

    #Initiate node for controlling joint1 and joint2 positions.
    rospy.init_node('acrobot_control_node', anonymous=True)

    #Define publishers for each joint position controller commands.
    # rospy.Subscriber('/rrbot/joint', Float64MultiArray, listen_joint_state)
    pub2 = rospy.Publisher('/rrbot/joint2_torque_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(100) #100 Hz

    #While loop to have joints follow a certain position, while rospy is not shutdown.
    i = 0
    while not rospy.is_shutdown():
        Acrobot collocated control
        (H, C, B) = acrobot.manipulator_dynamics(q, q_dot)

        H22_bar = H[1,1] - H[1,0]/H[0,0]*H[0,1]
        C2_bar = C[1] - H[1,0)]/H[0,0)]*C[0]
        
        des_q2 = 2*alpha/pi*atan(q_dot[0)]) 
        v2 = des_q2_ddot + kd*(des_q2_dot - q_dot[2]) + kp*(des_q2 - q[2])

        u = array([[0], [H22_bar*v2 + C2_bar]])

        #Have each joint follow a sine movement of sin(i/100).
        # u = sin(i/100.)

        #Publish the same sine movement to each joint.
        pub2.publish(u)

        i = i+1 #increment i

        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: acrobot_control_publisher()
    except rospy.ROSInterruptException: pass
