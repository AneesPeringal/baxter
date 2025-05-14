
"""
A demon to test the accuracy of the position get by the 3D camera
#!/home/binzhao/miniconda3/envs/spinningup/bin/python3.6 
"""
import sys
import copy
import tf
import rospy
import numpy as np
import time

import math
import rospkg
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

 
 
class PickAndPlace(object):
    def __init__(self, limb, mocap_sync_file, hover_distance=0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
 
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._gripper.calibrate()
        print("Enabling robot... ")
        self._rs.enable()
        self.mocap_config = mocap_sync_file
 
        self.kin = baxter_kinematics(limb)
        self.listener = tf.TransformListener()
 
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")
 
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
 
    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles is not None:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
 
    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)
 
    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
 
    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
 
    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)
 
    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)


 
    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()
        # thought away cube
        self.gripper_open()
 
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

    # def get_synced_mocap(self, )


# def euler_to_quaternion(roll, pitch, yaw):

#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

#     return [qx, qy, qz, qw]   

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
    return q_w, q_x, q_y, q_z
 

 
# def quaternion_to_euler(x, y, z, w):
    
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         X = math.atan2(t0, t1)

#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         Y = math.asin(t2)

#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         Z = math.atan2(t3, t4)

#         return X, Y, Z

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
 
def main():
    
    
    
    rospy.init_node("ik_pick_and_place_demo")
    hover_distance = 0 # meters
    # Starting Joint angles for left arm    

    
    # starting_joint_angles_left = {'left_s0':  -0.76578,
    #                         'left_s1': -0.91326,
    #                         'left_e0': -0.047871,
    #                         'left_e1': 1.5704,
    #                         'left_w0': -0.029908,
    #                         'left_w1': -0.65738,
    #                         'left_w2': 0.052933}    

    starting_joint_angles_left = {'left_s0':  -0,
                            'left_s1': 0,
                            'left_e0': 0,
                            'left_e1': 0,
                            'left_w0': 0,
                            'left_w1': 0,
                            'left_w2': 0}

    starting_joint_angles_right = {'right_s0': 0,
                            'right_s1': 0,
                            'right_e0': 0,
                            'right_e1': 0,
                            'right_w0': 0,
                            'right_w1': 0,
                            'right_w2': 0}




    pnp_left = PickAndPlace('left', hover_distance)
    print(pnp_left._limb_name)
    pnp_right = PickAndPlace('right', hover_distance)

    print(pnp_left.kin.forward_position_kinematics(starting_joint_angles_left))

    # pnp_left.move_to_start(starting_joint_angles_left)
    # pnp_right.move_to_start(starting_joint_angles_right)
    # An orientation for gripper fingers to be overhead and parallel to the obj

    # Move to the desired starting angles
    # pnp.move_to_start(starting_joint_angles)
    # string = input("start the demo: ")

    # time.sleep(5)
    try:
        while True:
            stage = int(input("enter stage to go to: "))
            if stage == 0:
                pnp_left._guarded_move_to_joint_position(starting_joint_angles_left)
                pnp_right._guarded_move_to_joint_position(starting_joint_angles_right)
                # time.sleep(2)
            elif stage == 1:
                pnp_left._guarded_move_to_joint_position(first_joint_angles_left)
                pnp_right._guarded_move_to_joint_position(first_joint_angles_right)
                # time.sleep(2)
            elif stage == 2:
                pnp_left._guarded_move_to_joint_position(second_joint_angles_left)
                pnp_right._guarded_move_to_joint_position(second_joint_angles_right)
                # time.sleep(2)
            elif stage == 3:
                pnp_left._guarded_move_to_joint_position(third_joint_angles_left)
                pnp_right._guarded_move_to_joint_position(third_joint_angles_right)
                # time.sleep(2)
            else:
                continue
    except KeyboardInterrupt:
        print("loop interuppted")

    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
