#!/usr/bin/python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from rcom_msgs.msg import rcom

counter = 0
t = 2.         # period in seconds
f = 200        # frequency
w = 2*np.pi/f  #

def deviation(ax: float, ay: float):  # ax, ay: amplitude in m
    dx = ax*np.cos(float(i)*w)
    dy = ay*np.sin(float(i)*w)
    global counter
    counter += 1
    return dx, dy

# publish sinusoidal rcom delta
rcom_pub = rospy.Publisher("rcom/p_trocar", Pose, queue_size=1)

rcom0_init = False
rcom0 = rcom()

# subscribe to fetch initial rcom position
def rCoMCB(msg: rcom):
    global rcom0_init
    global rcom0
    if not rcom0_init:
        rcom0 = msg
        rcom0_init = True

rcom_sub = rospy.Subscriber("h_rcom_vs/RCoM_ActionServer", rcom, , queue_size=1)

if __name__ == '__main__':


    while not rospy.is_shutdown:

        if rcom0_init:
            dx, dy = deviation(1., 1.)

            pose = Pose()
            pose.position.x = rcom0.p_trocar.position.x + dx
            pose.position.y = rcom0.p_trocar.position.y + dy
            pose.position.z = rcom0.p_trocar.position.z
            rcom_pub.publish(pose)

        rospy.sleep(rospy.Duration(float(t/f)))
