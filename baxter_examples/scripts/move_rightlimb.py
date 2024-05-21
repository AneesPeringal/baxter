#!/home/binzhao/miniconda3/envs/spinningup/bin/python3.6
"""
A demon to test the accuracy of the position get by the 3D camera
 
"""
import sys
import copy
import tf
import rospy
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
 
 
class PickAndPlace(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
	      
	self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
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
 
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")
 
    def ik_request(self, pose):
	print(pose)
 
        joints = self.kin.inverse_kinematics([pose.position.x, pose.position.y, pose.position.z],
                                             [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
	
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
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
 
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
 
 
def main():
 
    rospy.init_node("ik_pick_and_place_demo")
 
    limb = 'right'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm
    #starting_joint_angles = {'right_s0': -0.7243459305106948, 'right_s1': 0.49750126026966357, 'right_w0': 0.6996323424215224, 'right_w1': -1.4156648831918306, 'right_w2': -2.531262267096771, 'right_e0': 1.9571349486847451, 'right_e1': 1.5399539691931385}

    
    pnp = PickAndPlace(limb, hover_distance)
    #pnp.move_to_start(starting_joint_angles)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.671590815779,
                             y=0.735649402929,
                             z=-0.0837916180073,
                             w=-0.00036286450832011)
 
    # Move to the desired starting angles
    #pnp.move_to_start(starting_joint_angles)
    idx = 0
    pose = Pose(position=Point(x=0.57, y=-0.329, z=-0.481),
                    orientation=overhead_orientation)
 
    pnp._servo_to_pose(pose)

    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
