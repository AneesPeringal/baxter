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
pub = rospy.Publisher('/FK_pose', PoseStamped, queue_size = 10)
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
    

def callback(data):
    # in the order s0, s1, e0, e1, w0, w1, w2
    global current_angle
    try:
        joint_position = list(data.position)
        current_angle = [joint_position[4], joint_position[5], joint_position[2], joint_position[3], joint_position[6], joint_position[7], joint_position[8]]
        publish()

    except:
        pass


def kdl_rot2quat(rotm):

    r = PyKDL.Rotation(rotm[0,0], rotm[0,1], rotm[0,2],
                        rotm[1,0], rotm[1,1], rotm[1,2],
                        rotm[2,0], rotm[2,1], rotm[2,2])
    q = r.GetQuaternion()
    return q


def g_tr_callback(data):
    global tracking_pose
    tracking_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    

def main():
    global tracking_pose
    data = spio.loadmat("yz-circle_trajectory.mat");
    T_list = data['T_circular']
    transforms = [T_list[0,i] for i in range(T_list.shape[1])]
    poses = np.zeros((10,7))
    for i in range(len(transforms)):
        poses[i,0:3] = transforms[i][:3,3]
        r = transforms[i][:3,:3]
        q = kdl_rot2quat(r)
        poses[i,3:7] = [q[1], q[2], q[3], q[0]]
    print(poses)
        
  
    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
