
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
# from scipy.spatial.transform import Rotation
 
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
    def __init__(self, limb, hover_distance=0.15, verbose=True):
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
 
        joints = self.kin.inverse_kinematics([pose.position.x, pose.position.y, pose.position.z],[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
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


def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]   
 
def quaternion_to_euler(x, y, z, w):
    
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z
 
def main():
 
    rospy.init_node("ik_pick_and_place_demo")
 
    limb = 'left'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm
    starting_joint_angles_left = {'left_w0': 0.6699952259595108,
                           'left_w1': 1.030009435085784,
                           'left_w2': -0.4999997247485215,
                            'left_e0': -1.189968899785275,
                            'left_e1': 1.9400238130755056,
                            'left_s0': -0.08000397926829805,
                            'left_s1': -0.9999781166910306}

    starting_joint_angles_right = {'right_s0': 0.05829126993964572,
                            'right_s1': -0.96027197321626,
                            'right_w0': -0.6335340653966759,
                            'right_w1': 0.9637234299890112,
                            'right_w2': 0.4195437454866607,
                            'right_e0': 1.2103108416415915,
                            'right_e1': 1.9297478311598506}

    


    pnp_left = PickAndPlace(limb, hover_distance)
    pnp_right = PickAndPlace('right', hover_distance)
    # pnp_left.move_to_start(starting_joint_angles_left)
    # pnp_right.move_to_start(starting_joint_angles_right)
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
        roll_left, pitch_left, yaw_left = quaternion_to_euler(current_pose_left['orientation'].x, current_pose_left['orientation'].y, current_pose_left['orientation'].z, current_pose_left['orientation'].w)
        print("roll = ",roll_left, "\n")
        print("pitch = ", pitch_left, "\n")
        print("yaw = ", yaw_left, "\n")
        # print("Current pose left= ", current_pose_left)
        # print("\n")
        # current_pose_right = pnp_right._limb.endpoint_pose()
        trans_right, rot_right = pnp_left.get_tf_start_to_end_frames('Robot_1/base_link', 'base')
    
        current_orientation_right = Quaternion(
                                    x = rot_right[0],
                                    y = rot_right[1],
                                    z = rot_right[2],
                                    w = rot_right[3])
        # print("Current pose right= ", current_pose_right)
        # print("\n")
        roll_right, pitch_right, yaw_right = quaternion_to_euler(rot_right[0], rot_right[1], rot_right[2], rot_right)

        x_increment_left = input("x increment for left arm = ")
        y_increment_left = input("y increment for left arm = ")
        z_increment_left = input("z increment for left arm = ")
        roll_increment_left = input("roll increment for left arm = ")
        pitch_increment_left = input("pitch increment for left arm = ")
        yaw_increment_left = input("yaw increment for left arm = ")

        x_left, y_left, z_left, w_left = euler_to_quaternion(roll_left+roll_increment_left, pitch_left+pitch_increment_left, yaw_left +yaw_increment_left)
        new_orientation_left = Quaternion(
                                    x = x_left,
                                    y = y_left,
                                    z = z_left,
                                    w = w_left)

        x_increment_right = input("x increment for right arm = ")
        y_increment_right = input("y increment for right arm = ")
        z_increment_right = input("z increment for right arm = ")
        roll_increment_right = input("roll increment for right arm = ")
        pitch_increment_right = input("pitch increment for right arm = ")
        yaw_increment_right = input("yaw increment for right arm = ")
        x_right, y_right, z_right, w_right = euler_to_quaternion(roll_right+roll_increment_right, pitch_right+pitch_increment_right, yaw_right +yaw_increment_right)
        new_orientation_right = Quaternion(
                                    x = x_right,
                                    y = y_right,
                                    z = z_right,
                                    w = w_right)
        pose_left = Pose(position=Point(x=current_pose_left['position'].x+float(x_increment_left), y= current_pose_left['position'].y+float(y_increment_left), z= current_pose_left['position'].z+float(z_increment_left)),
                        orientation=new_orientation_left)
        print("new pose left = ",pose_left)
        pose_right = Pose(position=Point(x= trans_right[0]+float(x_increment_right), y= trans_right[1]+float(y_increment_right), z= trans_right[2]+float(z_increment_right)),
                        orientation=new_orientation_right)
        print("new pose right = ",pose_right)
        joint_angles_left = pnp_left.ik_request(pose_left)
        print(joint_angles_left)
        joint_angles_right = pnp_right.ik_request(pose_right)
        print(joint_angles_right)
        # try:
        #     # pnp_left._guarded_move_to_joint_position(joint_angles_left)
        #     # pnp_right._guarded_move_to_joint_position(joint_angles_right)
        # except:
        #     print("did not converge \n")

    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
