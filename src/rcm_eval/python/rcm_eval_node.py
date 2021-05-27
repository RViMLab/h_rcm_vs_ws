#!/usr/bin/python3

import os
import pandas as pd
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rcm_msgs.msg import rcm, rcmActionFeedback
from h_vs.msg import pairwise_distance

# dataframes to hold measurements
pairwise_distance_df = pd.DataFrame(columns=["time", "pairwise_distance"])
twist_df = pd.DataFrame(columns=["time", "Twist"])
h_rcm_vs_feedback_df = pd.DataFrame(columns=["time", "rcmFeedback"])
rcm_state_df = pd.DataFrame(columns=["time", "rcm"])
joint_state_df = pd.DataFrame(columns=["time", "JointState"])

# callbacks
def pairwiseDistanceCB(msg: pairwise_distance):
    global pairwise_distance_df
    pairwise_distance_df = pairwise_distance_df.append(
        {
            "time": rospy.Time.now().to_sec(), 
            "pairwise_distance": msg
        }, ignore_index=True
    )

def twistCB(msg: Twist):
    global twist_df
    twist_df = twist_df.append(
        {
            "time": rospy.Time.now().to_sec(),
            "Twist": msg
        }, ignore_index=True
    )

def hRCMVSFeedbackCB(msg: rcmActionFeedback):
    global h_rcm_vs_feedback_df
    h_rcm_vs_feedback_df = h_rcm_vs_feedback_df.append(
        {
            "time": rospy.Time.now().to_sec(),
            "rcmFeedback": msg.feedback
        }, ignore_index=True
    )

def rCoMStateCB(msg: rcm):
    global rcm_state_df
    rcm_state_df = rcm_state_df.append(
        {
            "time": rospy.Time.now().to_sec(),
            "rcm": msg
        }, ignore_index=True
    )

def jointStateCB(msg: JointState):
    global joint_state_df
    joint_state_df = joint_state_df.append(
        {
            "time": rospy.Time.now().to_sec(),
            "JointState": msg
        }, ignore_index=True
    )


if __name__ == '__main__':
    rospy.init_node("rcm_eval_node")

    # subscribe to
    #   - /lbr/visual_servo/pairwise_distance       measure visual error minimization
    #   - /lbr/visual_servo/twist                   measure error minimization
    #   - /lbr/h_rcm_vs/RCM_ActionServer/feedback   measure rcm deviation msg.errors.p_trocar
    #   - /lbr/RCM_ActionServer/state               measure rcm tip msg.state.task.values
    #   - /lbr/joint_states                         measure joint values
    pairwise_distance_sub = rospy.Subscriber("visual_servo/pairwise_distance", pairwise_distance, pairwiseDistanceCB)
    twist_sub = rospy.Subscriber("visual_servo/twist", Twist, twistCB)
    h_rcm_vs_feedback_sub = rospy.Subscriber("h_rcm_vs/RCM_ActionServer/feedback", rcmActionFeedback, hRCMVSFeedbackCB)
    rcm_state_sub = rospy.Subscriber("h_rcm_vs/RCM_ActionServer/state", rcm, rCoMStateCB)
    joint_state_sub = rospy.Subscriber("joint_states", JointState, jointStateCB)

    rospy.spin()

    rospy.loginfo("Saving measurements to {}".format(os.getcwd()))

    pairwise_distance_df.to_pickle("pairwise_distance.pkl", protocol=2)  # support read by python2
    twist_df.to_pickle("twist.pkl", protocol=2)
    h_rcm_vs_feedback_df.to_pickle("h_rcm_vs_feedback.pkl", protocol=2)
    rcm_state_df.to_pickle("rcm_state.pkl", protocol=2)
    joint_state_df.to_pickle("joint_states.pkl", protocol=2)
