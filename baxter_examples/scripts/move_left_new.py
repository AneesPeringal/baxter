
"""
A demon to test the accuracy of the position get by the 3D camera
#!/home/binzhao/miniconda3/envs/spinningup/bin/python3.6 
"""
import sys
import copy
import tf
import rospy
import numpy as np

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
 
    def get_tf_start_to_end_frames(self, start_frame, end_frame):
 
        start_frame = "/" + start_frame
        end_frame = "/" + end_frame

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
    roll = 0.7
    pitch = 0.5
    yaw = 0.2
    
    x,y,z,w = euler_to_quaternion(roll, pitch, yaw)
    print(quaternion_to_euler(x,y,z,w))
    rospy.init_node("ik_pick_and_place_demo")
 
    limb = 'left'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm
    # starting_joint_angles_left = {'left_w0': 0.6699952259595108,
    #                        'left_w1': 1.030009435085784,
    #                        'left_w2': -0.4999997247485215,
    #                         'left_e0': -1.189968899785275,
    #                         'left_e1': 1.9400238130755056,
    #                         'left_s0': -0.08000397926829805,
    #                         'left_s1': -0.9999781166910306}

    # starting_joint_angles_right = {'right_s0': 0.05829126993964572,
    #                         'right_s1': -0.96027197321626,
    #                         'right_w0': -0.6335340653966759,
    #                         'right_w1': 0.9637234299890112,
    #                         'right_w2': 0.4195437454866607,
    #                         'right_e0': 1.2103108416415915,
    #                         'right_e1': 1.9297478311598506}

    

    starting_joint_angles_left = {'left_e0':  -0.05292233718204677,
                            'left_e1': 2.2741265180401258,
                            'left_s0': -0.6830049458059805,
                            'left_s1': -1.1389807350049197,
                            'left_w0': 0.08130098175792692,
                            'left_w1': -1.1094516048381255,
                            'left_w2': -0.008053399136398421}

    starting_joint_angles_right = {'right_e0':  0.47131559707779336,
                            'right_e1': 2.171349805251803,
                            'right_s0': 0.11850001586414821,
                            'right_s1': -1.0043739208679747,
                            'right_w0': -0.22089323345549958,
                            'right_w1': -1.07953897947436,
                            'right_w2': -0.10967962633380708}


    pnp_left = PickAndPlace(limb, hover_distance)
    pnp_right = PickAndPlace('right', hover_distance)
    trans, rot = pnp_left.get_tf_start_to_end_frames('left_hand','base')
    # trans, rot = pp_left.get_tf_start_to_end_frames('Robot_1/base_link', 'base')
    print(rot)
    pnp_left.move_to_start(starting_joint_angles_left)
    pnp_right.move_to_start(starting_joint_angles_right)
    # An orientation for gripper fingers to be overhead and parallel to the obj

    # Move to the desired starting angles
    # pnp.move_to_start(starting_joint_angles)
    while True:
        print("collecting current_pose")
        current_pose_left = pnp_left._limb.endpoint_pose()
    

        current_orientation_left = Quaternion(
                                    x = current_pose_left['orientation'].x,
                                    y = current_pose_left['orientation'].y,
                                    z = current_pose_left['orientation'].z,
                                    w = current_pose_left['orientation'].w)
        # r = Rotation.from_quat([current_orientation_left.x, current_orientation_left.y, current_orientation_left.z, current_orientation_left.w])

        roll_left, pitch_left, yaw_left = quaternion_to_euler(current_pose_left['orientation'].x, current_pose_left['orientation'].y, current_pose_left['orientation'].z, current_pose_left['orientation'].w)
        # print("roll = ",roll_left, "\n")
        # print("pitch = ", pitch_left, "\n")
        # print("yaw = ", yaw_left, "\n")
        # print("Current pose left= ", current_pose_left)
        # print("\n")
        current_pose_right = pnp_right._limb.endpoint_pose()
        current_orientation_right = Quaternion(
                                    x = current_pose_right['orientation'].x,
                                    y = current_pose_right['orientation'].y,
                                    z = current_pose_right['orientation'].z,
                                    w = current_pose_right['orientation'].w)
        print("Current pose left = ", current_pose_left)
        print("Current pose right= ", current_pose_right)
        # print("\n")
        roll_right, pitch_right, yaw_right = quaternion_to_euler(current_pose_right['orientation'].x, current_pose_right['orientation'].y, current_pose_right['orientation'].z, current_pose_right['orientation'].w)

        x_left_new = input("x increment for left arm = ")
        y_left_new = input("y increment for left arm = ")
        z_left_new = input("z increment for left arm = ")
        
        x_quat_left_new = input("x quaternion for left arm = ")
        y_quat_left_new = input("y quaternion for left arm = ")
        z_quat_left_new = input("z quaternion for left arm = ")
        w_quat_left_new = input("w quaternion for left arm = ")


        # x_left, y_left, z_left, w_left = euler_to_quaternion(roll_left_new, pitch_left_new, yaw_left_new)
        new_orientation_left = Quaternion(
                                    x = current_orientation_left.x + x_quat_left_new,
                                    y = current_orientation_left.y + y_quat_left_new,
                                    z = current_orientation_left.z + z_quat_left_new,
                                    w = current_orientation_left.w + w_quat_left_new)

        x_right_new = input("x increment for right arm = ")
        y_right_new = input("y increment for right arm = ")
        z_right_new = input("z increment for right arm = ")
        
        x_quat_right_new = input("x quaternion for right arm = ")
        y_quat_right_new = input("y quaternion for right arm = ")
        z_quat_right_new = input("z quaternion for right arm = ")
        w_quat_right_new = input("w quaternion for right arm = ")
        # x_right, y_right, z_right, w_right = euler_to_quaternion(roll_right_new, pitch_right_new, yaw_right_new)
        # x_right, y_right, z_right, w_right = euler_to_quaternion(-0.6326, 0.6344, 2.1545)
        try_again = input("try")
        new_orientation_right = Quaternion(
                                    x = current_orientation_left.x  + x_quat_right_new,
                                    y = current_orientation_right.y + y_quat_right_new,
                                    z = current_orientation_right.z + z_quat_right_new,
                                    w = current_orientation_right.w + w_quat_right_new)
                                    
        pose_left = Pose(position=Point(x=current_pose_left["position"].x + float(x_left_new), y= current_pose_left["position"].y +float(y_left_new), z= current_pose_left["position"].z +float(z_left_new)),
                        orientation=new_orientation_left)
        print("new pose left = ",pose_left)
        pose_right = Pose(position=Point(x= current_pose_right["position"].x + float(x_right_new), y= current_pose_right["position"].y +float(y_right_new), z= current_pose_right["position"].z + float(z_right_new)),
                        orientation=new_orientation_right)
        print("new pose right = ",pose_right)

        try:
            joint_angles_left = pnp_left.ik_request(pose_left)
            pnp_left._guarded_move_to_joint_position(joint_angles_left)

        except:
            print("did not converge left \n")
        try:
            joint_angles_right = pnp_right.ik_request(pose_right)

            pnp_right._guarded_move_to_joint_position(joint_angles_right)
        except:
            print("did not converge right \n")

    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
