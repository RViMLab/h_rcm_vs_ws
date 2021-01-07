import pandas as pd
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rcom_msgs.msg import rcom, rcomFeedback

# dataframes to hold measurements
mean_pairwise_distance_df = pd.DataFrame(columns=["mean_pairwise_distance"])
twist_df = pd.DataFrame(columns=["linear", "angular"])
h_rcom_vs_feedback_df = pd.DataFrame(columns=["p_trocar_error"])
rcom_state_df = pd.DataFrame(columns=["tip"])

# callbacks
def meanPairwiseDistanceCB(msg: Float64):
    mean_pairwise_distance_df = mean_pairwise_distance_df.append({"mean_pairwise_distance": msg})

def twistCB(msg: Twist):
    twist_df = twist_df.append({"linear": msg.linear, "angular": msg.angular})

def hRCoMVSFeedbackCB(msg: rcomFeedback):
    h_rcom_vs_feedback_df = h_rcom_vs_feedback_df.append({"p_trocar_error": msg.errors.p_trocar.position})

def rCoMStateCB(msg: rcom):
    rcom_state_df = rcom_state_df.append({"p_trocar": msg.p_trocar.position, "task": msg.task.values})

# subscribe to
#   - /lbr/visual_servo/mean_pairwise_distance   measure visual error minimization
#   - /lbr/visual_servo/twist                    measure error minimization
#   - /lbr/h_rcom_vs/RCoM_ActionServer/feedback  measure rcom deviation msg.errors.p_trocar
#   - /lbr/RCoM_ActionServer/state               measure rcom tip msg.state.task.values
mean_pairwise_distance_sub = rospy.Subscriber("visual_servo/mean_pairwise_distance", Float64, meanPairwiseDistanceCB)
twist_sub = rospy.Subscriber("visual_servo/twist", Twist, twistCB)
h_rcom_vs_feedback_sub = rospy.Subscriber("h_rcom_vs/RCoM_ActionServer/feedback", rcomFeedback, hRCoMVSFeedbackCB)
rcom_state_sub = rospy.Subscriber("RCoM_ActionServer/state", rcom, rCoMStateCB)

# experiments
#   - visual servo
#     - random perturbation
#     - random perturbation under rcom deviation
#   - camera displacement

if __name__ == '__main__':
    rospy.init_node("rcom_eval_node")

    rospy.spin()
