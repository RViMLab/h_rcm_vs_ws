#!/usr/bin/python3

import os
import pandas as pd
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rcm_msgs.msg import rcm, rcmActionFeedback
from h_vs.msg import pairwise_distance

# dataframes to hold measurements
pairwise_distance_df = pd.DataFrame(columns=["pairwise_distance"])
twist_df = pd.DataFrame(columns=["Twist"])
h_rcm_vs_feedback_df = pd.DataFrame(columns=["rcmFeedback"])
rcm_state_df = pd.DataFrame(columns=["rcm"])

# callbacks
def meanPairwiseDistanceCB(msg: pairwise_distance):
    global pairwise_distance_df
    pairwise_distance_df = pairwise_distance_df.append({"pairwise_distance": msg}, ignore_index=True)

def twistCB(msg: Twist):
    global twist_df
    twist_df = twist_df.append({"Twist": msg}, ignore_index=True)

def hRCMVSFeedbackCB(msg: rcmActionFeedback):
    global h_rcm_vs_feedback_df
    h_rcm_vs_feedback_df = h_rcm_vs_feedback_df.append({"rcmFeedback": msg.feedback}, ignore_index=True)

def rCoMStateCB(msg: rcm):
    global rcm_state_df
    rcm_state_df = rcm_state_df.append({"rcm": msg}, ignore_index=True)

# subscribe to
#   - /lbr/visual_servo/pairwise_distance   measure visual error minimization
#   - /lbr/visual_servo/twist                    measure error minimization
#   - /lbr/h_rcm_vs/RCM_ActionServer/feedback  measure rcm deviation msg.errors.p_trocar
#   - /lbr/RCM_ActionServer/state               measure rcm tip msg.state.task.values
pairwise_distance_sub = rospy.Subscriber("visual_servo/pairwise_distance", Float64, meanPairwiseDistanceCB)
twist_sub = rospy.Subscriber("visual_servo/twist", Twist, twistCB)
h_rcm_vs_feedback_sub = rospy.Subscriber("h_rcm_vs/RCM_ActionServer/feedback", rcmActionFeedback, hRCMVSFeedbackCB)
rcm_state_sub = rospy.Subscriber("h_rcm_vs/RCM_ActionServer/state", rcm, rCoMStateCB)

# experiments
#   - visual servo
#     - random perturbation
#     - random perturbation under rcm deviation
#   - camera displacement

if __name__ == '__main__':
    rospy.init_node("rcm_eval_node")

    rospy.spin()

    rospy.loginfo("Saving measurements to {}".format(os.getcwd()))

    pairwise_distance_df.to_pickle("pairwise_distance.pkl")
    twist_df.to_pickle("twist.pkl")
    h_rcm_vs_feedback_df.to_pickle("h_rcm_vs_feedback.pkl")
    rcm_state_df = rcm_state_df.to_pickle("rcm_state.pkl")
