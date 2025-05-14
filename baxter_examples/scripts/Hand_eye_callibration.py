## Get the data required for the hand-eye callibration

## Finding the difference between pykdl and robotics system toolbox interms of the definition of the frames

import sys
import copy
import tf
import rospy
import numpy as np
import scipy.io as spio

import math
import rospkg
import PyKDL
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from sensor_msgs.msg import JointState

from std_msgs.msg import (
    Header,
    Float64MultiArray,
    String,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

rospy.init_node("ik_pick_and_place_demo")
rate = rospy.Rate(10)
pub_left = rospy.Publisher('/FK_pose_left', PoseStamped, queue_size = 10)
pub_right = rospy.Publisher('/FK_pose_right', PoseStamped, queue_size = 10)

current_angle =None
desired_angle = None
tracking_pose = None


class PickAndPlace(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
 
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
 
        self.kin = baxter_kinematics(limb)
        self.listener = tf.TransformListener()

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles is not None:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    




    def fk_request(self, values):
        pose = self.kin.forward_position_kinematics(values)


    


def callback(data):
    # in the order s0, s1, e0, e1, w0, w1, w2
    global current_angle
    try:
        joint_position = list(data.position)
        current_angle = [joint_position[4], joint_position[5], joint_position[2], joint_position[3], joint_position[6], joint_position[7], joint_position[8]]
        publish()
        # joint_position = list(data.position)
        # current_joint_position = [joint_position[4], joint_position[5], joint_position[2], joint_position[3], joint_position[6], joint_position[7], joint_position[8]]
        # desired_joint_position = arg
        # difference = list(np.asarray(current_joint_position) - np.asarray(desired_joint_position))
        # msg = Float64MultiArray()
        # msg.data = difference
        # pub.publish(msg)
        # rate.sleep()
    except:
        pass



def main():
    pnp_left = PickAndPlace('left')
    pnp_right = PickAndPlace('right')
    starting_joint_angles_left = {'left_s0':  -0.58664,
                            'left_s1': -1.137,
                            'left_e0': -0.034547,
                            'left_e1': 2.0042,
                            'left_w0': 0.21857,
                            'left_w1': -0.87636,
                            'left_w2': -0.12647}

    starting_joint_angles_right = {'right_s0':  0.22123,
                            'right_s1': -1.0741,
                            'right_e0': 0.50658,
                            'right_e1': 1.984,
                            'right_w0': -0.14492,
                            'right_w1': -0.83544,
                            'right_w2': -0.13709}
    pnp_left._guarded_move_to_joint_position(starting_joint_angles_left)
    pnp_right._guarded_move_to_joint_position(starting_joint_angles_right)

    input("define_rigid_body?")
    


    while not rospy.is_shutdown():
        current_joint_left = pnp_left._limb.joint_angles()
        current_pose_left = pnp_left.kin.forward_position_kinematics(current_joint_left)
        # print(current_pose)

        pose_msg_left = PoseStamped()

        pose_msg_left.header = Header()
        pose_msg_left.header.stamp = rospy.Time.now()
        pose_msg_left.header.frame_id = "base"

        pose_msg_left.pose.position.x = current_pose_left[0]
        pose_msg_left.pose.position.y = current_pose_left[1]
        pose_msg_left.pose.position.z = current_pose_left[2]

        pose_msg_left.pose.orientation.x = current_pose_left[3]
        pose_msg_left.pose.orientation.y = current_pose_left[4]
        pose_msg_left.pose.orientation.z = current_pose_left[5]
        pose_msg_left.pose.orientation.w = current_pose_left[6]

        pub_left.publish(pose_msg_left)
        current_joint_right = pnp_right._limb.joint_angles()
        current_pose_right = pnp_right.kin.forward_position_kinematics(current_joint_right)
        # print(current_pose)

        pose_msg_right = PoseStamped()

        pose_msg_right.header = Header()
        pose_msg_right.header.stamp = rospy.Time.now()
        pose_msg_right.header.frame_id = "base"

        pose_msg_right.pose.position.x = current_pose_right[0]
        pose_msg_right.pose.position.y = current_pose_right[1]
        pose_msg_right.pose.position.z = current_pose_right[2]

        pose_msg_right.pose.orientation.x = current_pose_right[3]
        pose_msg_right.pose.orientation.y = current_pose_right[4]
        pose_msg_right.pose.orientation.z = current_pose_right[5]
        pose_msg_right.pose.orientation.w = current_pose_right[6]

        pub_right.publish(pose_msg_right)
        rate.sleep()
    
    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
