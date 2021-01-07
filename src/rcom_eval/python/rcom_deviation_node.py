#!/usr/bin/python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from rcom_msgs.msg import rcom

T  = 1.          # period
t  = 0.         # current time
f  = 1/T        # frequency
dt = 0.01       # dt
amplitude_x = 1.
amplitude_y = 1.

def deviation(ax: float, ay: float):  # ax, ay: amplitude in m
    global t
    global f
    dx = ax*np.cos(2*np.pi*f*t)
    dy = ay*np.sin(2*np.pi*f*t)
    t += dt
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

rcom_sub = rospy.Subscriber("h_rcom_vs/RCoM_ActionServer", rcom, rCoMCB, queue_size=1)

if __name__ == '__main__':
    rospy.init_node("rcom_deviation_node")

    T  = rospy.get_param("rcom_deviation_node/period")
    dt = rospy.get_param("rcom_deviation_node/dt")
    amplitude_x = rospy.get_param("rcom_deviation_node/amplitude_x")
    amplitude_y = rospy.get_param("rcom_deviation_node/amplitude_y")
    f = float(1/T)

    r = rospy.Rate(float(1/dt))
    while not rospy.is_shutdown():

        if rcom0_init:
            dx, dy = deviation(amplitude_x, amplitude_y)

            pose = Pose()
            pose.position.x = rcom0.p_trocar.position.x + dx
            pose.position.y = rcom0.p_trocar.position.y + dy
            pose.position.z = rcom0.p_trocar.position.z
            rcom_pub.publish(pose)

        r.sleep()
