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
rate = rospy.Rate(100)

tracking_pose_left = None
tracking_pose_right = None

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
    
    def ik_request(self, pose):
     
        joints = self.kin.inverse_kinematics([pose.position.x, pose.position.y, pose.position.z],[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], )
        if self._limb_name == 'left':
            limb_joints = {'left_w0': joints[4],
                                'left_w1': joints[5],
                                'left_w2': joints[6],
                                'left_e0': joints[2],
                                'left_e1': joints[3],
                                'left_s0': joints[0],
                                'left_s1': joints[1]}
        elif self._limb_name == 'right':
            limb_joints = {'right_w0': joints[4],
                        'right_w1': joints[5],
                        'right_w2': joints[6],
                        'right_e0': joints[2],
                        'right_e1': joints[3],
                        'right_s0': joints[0],
                        'right_s1': joints[1]}
            
        
        return limb_joints

    def dict_joint(self,joint_array):
        if self._limb_name == 'left':
            limb_joints = {'left_w0': joint_array[4],
                                'left_w1': joint_array[5],
                                'left_w2': joint_array[6],
                                'left_e0': joint_array[2],
                                'left_e1': joint_array[3],
                                'left_s0': joint_array[0],
                                'left_s1': joint_array[1]}
        elif self._limb_name == 'right':
            limb_joints = {'right_w0': joint_array[4],
                        'right_w1': joint_array[5],
                        'right_w2': joint_array[6],
                        'right_e0': joint_array[2],
                        'right_e1': joint_array[3],
                        'right_s0': joint_array[0],
                        'right_s1': joint_array[1]}
            
        
        return limb_joints


    def fk_request(self, values):
        pose = self.kin.forward_position_kinematics(values)

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles is not None:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    
    def get_tf_start_to_end_frames(self):
        start_frame = "/base"
        end_frame = "/" + self._limb_name+"_gripper_base"

        trans, rot = None, None
        try:
            (trans, rot) = self.listener.lookupTransform(start_frame, end_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF start to end not ready YET...")
            pass
 
        return trans, rot




def kdl_rot2quat(rotm):

    r = PyKDL.Rotation(rotm[0,0], rotm[0,1], rotm[0,2],
                        rotm[1,0], rotm[1,1], rotm[1,2],
                        rotm[2,0], rotm[2,1], rotm[2,2])
    q = r.GetQuaternion()
    return q

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion (q_w, q_x, q_y, q_z)
    Args:
    roll: Rotation around the x-axis in radians
    pitch: Rotation around the y-axis in radians
    yaw: Rotation around the z-axis in radians
    Returns:
    q_w, q_x, q_y, q_z: components of the quaternion
    """
    # Calculate the quaternion components
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
 
    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy
    return q_x, q_y, q_z,q_w

def quaternion_to_euler(q_w, q_x, q_y, q_z):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    Roll is rotation around x-axis (phi)
    Pitch is rotation around y-axis (theta)
    Yaw is rotation around z-axis (psi)
    Args:
    q_w, q_x, q_y, q_z: components of the quaternion
 
    Returns:
    roll, pitch, yaw: Euler angles in radians
    """
    # Calculate roll (x-axis rotation)
    sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Calculate pitch (y-axis rotation)
    sinp = 2 * (q_w * q_y - q_z * q_x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    # Calculate yaw (z-axis rotation)
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
def tr_left_callback(data):
    global tracking_pose_left
    tracking_pose_left = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    
def tr_right_callback(data):
    global tracking_pose_right
    tracking_pose_right = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

def own_move_to_joint(left_limb, right_limb,joint_angles_left, joint_angles_right, duration):
        ## smoothly move to a desired joint angle together
    left_joint = left_limb.joint_angles()
    right_joint = right_limb.joint_angles()

    joint_names_left = joint_angles_left.keys()
    joint_names_right = joint_angles_right.keys()
    start_array_left = np.array([left_joint[j] for j in joint_names_left])
    target_array_left = np.array([joint_angles_left[j] for j in joint_names_left])

    start_array_right = np.array([right_joint[j] for j in joint_names_right])
    target_array_right = np.array([joint_angles_right[j] for j in joint_names_right])


    n_steps = int(duration*100) ## make this correct later
    for i in range(n_steps):
        alpha = float(i)/(n_steps-1)
        interp_array_left = (1-alpha)*start_array_left+alpha*target_array_left
        interp_array_right = (1-alpha)*start_array_right + alpha*target_array_right
        interp_dict_left = dict(zip(joint_names_left, interp_array_left))
        interp_dict_right = dict(zip(joint_names_right, interp_array_right))
        right_limb.set_joint_positions(interp_dict_right)
        left_limb.set_joint_positions(interp_dict_left)
        rate.sleep()
    left_limb.set_joint_positions(joint_angles_left)
    right_limb.set_joint_positions(joint_angles_right)


def main():
    global tracking_pose_left
    global tracking_pose_right
    data = spio.loadmat("both_arms_circle.mat");
    T_left = data['T_left']
    T_right = data['T_right']
    transforms_left = [T_left[0,i] for i in range(T_left.shape[1])]
    transforms_right = [T_right[0,i] for i in range(T_right.shape[1])]
    poses_left = np.zeros((len(transforms_left),7))
    poses_right = np.zeros((len(transforms_right), 7))
    for i in range(len(transforms_left)):
        poses_left[i,0:3] = transforms_left[i][:3,3]
        poses_right[i,0:3] = transforms_right[i][:3,3]
        r = transforms_left[i][:3,:3]
        q = kdl_rot2quat(r)
        poses_left[i,3:7] = [q[1], q[2], q[3], q[0]]
        r = transforms_right[i][:3,:3]
        q = kdl_rot2quat(r)
        poses_right[i,3:7] = [q[1], q[2], q[3], q[0]]

    rospy.Subscriber("/opti/left_arm/transformed_pose",PoseStamped, tr_left_callback)
    rospy.Subscriber("/opti/right_arm/transformed_pose",PoseStamped, tr_right_callback)
    # Starting Joint angles for left arm    
           
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

    pnp_left = PickAndPlace('left')

    pnp_right = PickAndPlace('right')
    keys = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    desired_angle = [starting_joint_angles_left['left_'+key] for key in keys]
    # print("Testing...")
    pnp_left._guarded_move_to_joint_position(starting_joint_angles_left)
    pnp_right._guarded_move_to_joint_position(starting_joint_angles_right)
    
    current_joint_left = pnp_right._limb.joint_angles()
    current_pose = pnp_right.kin.forward_position_kinematics(current_joint_left)
    print(current_pose)


    input("defined rigid_body")



    errors_left =np.array([])
    errors_right = np.array([])
    for i in range(len(poses_left)):
        desired_pose_left = poses_left[i,:]
        desired_pose_right = poses_right[i,:]
        # print(desired_pose)
        desired_joint_angles_left = pnp_left.kin.inverse_kinematics_LM(desired_pose_left[0:3], desired_pose_left[3:7])
        desired_joint_angles_right = pnp_right.kin.inverse_kinematics_LM(desired_pose_right[0:3], desired_pose_right[3:7])
        if desired_joint_angles_left == None or desired_joint_angles_right == None:
            continue
        # pnp_left._guarded_move_to_joint_position(pnp_left.dict_joint(desired_joint_angles_left))
        # pnp_right._guarded_move_to_joint_position(pnp_right.dict_joint(desired_joint_angles_right))


        print("error left", np.array(desired_pose_left) - tracking_pose_left)
        print("error right", np.array(desired_pose_right) - tracking_pose_right)
        np.append(errors_left,np.array(desired_pose_left) - tracking_pose_left)
        np.append(errors_right,np.array(desired_pose_right) - tracking_pose_right)
    
    # print(errors_left)
    # rmse_l = np.sqrt(np.mean(errors_left[0:3]**2))

    
  
    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
