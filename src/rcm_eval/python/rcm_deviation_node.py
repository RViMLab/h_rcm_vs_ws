#!/usr/bin/python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from rcm_msgs.msg import rcm

T  = 1.         # period
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

# publish sinusoidal rcm delta
rcm_pub = rospy.Publisher("rcm/p_trocar", Pose, queue_size=1)

rcm0_init = False
rcm0 = rcm()

# subscribe to fetch initial rcm position
def rCoMCB(msg: rcm):
    global rcm0_init
    global rcm0

    if not rcm0_init:
        rcm0 = msg
        rcm0_init = True

rcm_sub = rospy.Subscriber("h_rcm_vs/RCM_ActionServer", rcm, rCoMCB, queue_size=1)

if __name__ == '__main__':
    rospy.init_node("rcm_deviation_node")

    T  = rospy.get_param("rcm_deviation_node/period")
    dt = rospy.get_param("rcm_deviation_node/dt")
    amplitude_x = rospy.get_param("rcm_deviation_node/amplitude_x")
    amplitude_y = rospy.get_param("rcm_deviation_node/amplitude_y")
    f = float(1/T)

    r = rospy.Rate(float(1/dt))
    while not rospy.is_shutdown():

        if rcm0_init:
            dx, dy = deviation(amplitude_x, amplitude_y)

            pose = Pose()
            pose.position.x = rcm0.p_trocar.position.x + dx
            pose.position.y = rcm0.p_trocar.position.y + dy
            pose.position.z = rcm0.p_trocar.position.z
            rcm_pub.publish(pose)

        r.sleep()
