#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from math import sin,cos,atan,atan2,sqrt,fabs
from numpy import *
from sensor_msgs.msg import JointState

class Acrobot:
    _m = (1,2)
    _l = (1,2)
    _lc = (0.5,1)
    _Ic = (0.083,0.33)
    _g = 9.81
    _b = (.1, .1)

    def __init__(self, m, l, lc, Ic):
        self._m = m
        self._l = l
        self._lc = lc
        self._Ic = Ic
    
    def manipulator_dynamics(self, q, qd):
         # keep it readable:
        (m1,m2)=self._m
        (l1,l2)=self._l
        g=self._g
        (lc1,lc2)=self._lc
        (b1,b2)=self._b
        I1 = self._Ic[0] + m1*lc1**2
        I2 = self._Ic[1] + m2*lc2**2
        m2l1lc2 = m2*l1*lc2  # occurs often!

        c = cos(q[0:2,:])
        s = sin(q[0:2,:])
        s12 = sin(q[0,:]+q[1,:])

        h12 = I2 + m2l1lc2*c[1]
        H = array([[ I1 + I2 + m2*l1**2 + 2*m2l1lc2*c[1], h12],
                [h12, I2] ])

        C_ = array([[-2*m2l1lc2*s[1]*qd[1], -m2l1lc2*s[1]*qd[1]],
                [m2l1lc2*s[1]*qd[0], 0] ])
        
        G = g*array([[ m1*lc1*s[0] + m2*(l1*s[1]+lc2*s12)],
            [m2*lc2*s12] ])
            
        # accumate total C and add a damping term:
        C = C_.dot(qd) + G + array([[b1],[b2]])*qd
        print ('C_.dot(qd)', C_.dot(qd))
        print ('G', G)
        print ('C_.dot(qd) + G', C_.dot(qd) + G)
        B = array([[0], [1]])

        return (H, C, B)

def listen_joint_state(joint_state, (q, qd)):
    q[0] = 46 # joint_state.position[0]
    q[1] = joint_state.position[1]
    qd[0] = joint_state.velocity[0]
    qd[1] = joint_state.velocity[1]
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %f', q[0]) #, q[1])

#Define a RRBot joint positions publisher for joint controllers.
def acrobot_control_publisher():
    acrobot = Acrobot((1, 2), (1, 2), (0.5, 1), (0.083, 0.33))

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
    rospy.Subscriber('/rrbot/joint_states', JointState, listen_joint_state, (q, q_dot) )
    pub2 = rospy.Publisher('/rrbot/joint2_torque_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(100) #100 Hz

    #While loop to have joints follow a certain position, while rospy is not shutdown.
    i = 0
    while not rospy.is_shutdown():
        # Acrobot collocated control
        (H, C, B) = acrobot.manipulator_dynamics(q, q_dot)

        H22_bar = H[1,1] - H[1,0]/H[0,0]*H[0,1]
        C2_bar = C[1] - H[1,0]/H[0,0]*C[0]
        
        des_q2 = 2*alpha/pi*atan(q_dot[0]) 
        v2 = des_q2_ddot + kd*(des_q2_dot - q_dot[1]) + kp*(des_q2 - q[1])

        u = array([[0.0], [H22_bar*v2 + C2_bar]])
        #Have each joint follow a sine movement of sin(i/100).
        # u = sin(i/100.)

        #Publish the same sine movement to each joint.
        # print ('(H)', size(C))
        pub2.publish(u[1])

        i = i+1 #increment i

        rate.sleep() #sleep for rest of rospy.Rate(100)

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: acrobot_control_publisher()
    except rospy.ROSInterruptException: pass