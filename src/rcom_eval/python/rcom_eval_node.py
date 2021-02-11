#!/usr/bin/python3

import os
import pandas as pd
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rcom_msgs.msg import rcom, rcomActionFeedback

# dataframes to hold measurements
mean_pairwise_distance_df = pd.DataFrame(columns=["mean_pairwise_distance"])
twist_df = pd.DataFrame(columns=["Twist"])
h_rcom_vs_feedback_df = pd.DataFrame(columns=["rcomFeedback"])
rcom_state_df = pd.DataFrame(columns=["rcom"])

# callbacks
def meanPairwiseDistanceCB(msg: Float64):
    global mean_pairwise_distance_df
    mean_pairwise_distance_df = mean_pairwise_distance_df.append({"mean_pairwise_distance": msg}, ignore_index=True)

def twistCB(msg: Twist):
    global twist_df
    twist_df = twist_df.append({"Twist": msg}, ignore_index=True)

def hRCoMVSFeedbackCB(msg: rcomActionFeedback):
    global h_rcom_vs_feedback_df
    h_rcom_vs_feedback_df = h_rcom_vs_feedback_df.append({"rcomFeedback": msg.feedback}, ignore_index=True)

def rCoMStateCB(msg: rcom):
    global rcom_state_df
    rcom_state_df = rcom_state_df.append({"rcom": msg}, ignore_index=True)

# subscribe to
#   - /lbr/visual_servo/mean_pairwise_distance   measure visual error minimization
#   - /lbr/visual_servo/twist                    measure error minimization
#   - /lbr/h_rcom_vs/RCoM_ActionServer/feedback  measure rcom deviation msg.errors.p_trocar
#   - /lbr/RCoM_ActionServer/state               measure rcom tip msg.state.task.values
mean_pairwise_distance_sub = rospy.Subscriber("visual_servo/mean_pairwise_distance", Float64, meanPairwiseDistanceCB)
twist_sub = rospy.Subscriber("visual_servo/twist", Twist, twistCB)
h_rcom_vs_feedback_sub = rospy.Subscriber("h_rcom_vs/RCoM_ActionServer/feedback", rcomActionFeedback, hRCoMVSFeedbackCB)
rcom_state_sub = rospy.Subscriber("h_rcom_vs/RCoM_ActionServer/state", rcom, rCoMStateCB)

# experiments
#   - visual servo
#     - random perturbation
#     - random perturbation under rcom deviation
#   - camera displacement

if __name__ == '__main__':
    rospy.init_node("rcom_eval_node")

    rospy.spin()

    rospy.loginfo("Saving measurements to {}".format(os.getcwd()))

    mean_pairwise_distance_df.to_pickle("mean_pairwise_distance.pkl")
    twist_df.to_pickle("twist.pkl")
    h_rcom_vs_feedback_df.to_pickle("h_rcom_vs_feedback.pkl")
    rcom_state_df = rcom_state_df.to_pickle("rcom_state.pkl")
