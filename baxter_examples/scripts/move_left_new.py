
"""
A demon to test the accuracy of the position get by the 3D camera
#!/home/binzhao/miniconda3/envs/spinningup/bin/python3.6 
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
 
        joints = self.kin.inverse_kinematics([pose.position.x, pose.position.y, pose.position.z],
                                             [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        limb_joints = {'left_w0': joints[4],
                            'left_w1': joints[5],
                            'left_w2': joints[6],
                            'left_e0': joints[2],
                            'left_e1': joints[3],
                            'left_s0': joints[0],
                            'left_s1': joints[1]}
        
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

    def _recursive_servo_to_pose(self, pose, pose_init):
        while pose_init != pose:
            try:
                self._servo_to_pose(pose)
                pose_init = pose
            except:
                pose = Pose(position=Point(x=(pose.position.x- pose_init.position.x)/2, y= (pose.position.y - pose_init.position.y)/2, z= (pose.position.z - pose_init.position.z)/2),
                    orientation=overhead_orientation)
                self._recursive_servo_to_pose(pose, pose_init)


 
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

    def recursive_IK(self,pose_init, pose):
        joints = self.kin.inverse_kinematics(pose)
        try:
            limb_joints = {'left_w0': joints[4],
                            'left_w1': joints[5],
                            'left_w2': joints[6],
                            'left_e0': joints[2],
                            'left_e1': joints[3],
                            'left_s0': joints[0],
                            'left_s1': joints[1]}
            pose_init = pose
        except:
            pose = Pose(position=Point(x=(pose.position.x- pose_init.position)/2, y= (pose.position.y - pose_init.position.y)/2, z= (pose.position.z - pose_init.position.z)/2),
                    orientation=overhead_orientation)
            limb_joints = recursive_IK(pose_init,pose)
        return limb_joints, pose

        
 
 
def main():
 
    rospy.init_node("ik_pick_and_place_demo")
 
    limb = 'left'
    hover_distance = 0 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                           'left_w1': 1.030009435085784,
                           'left_w2': -0.4999997247485215,
                            'left_e0': -1.189968899785275,
                            'left_e1': 1.9400238130755056,
                            'left_s0': -0.08000397926829805,
                            'left_s1': -0.9999781166910306}
    joint_init = [starting_joint_angles['left_s0'],
                starting_joint_angles['left_s1'],
                starting_joint_angles['left_e0'],
                starting_joint_angles['left_e1'],
                starting_joint_angles['left_w0'],
                starting_joint_angles['left_w1'],
                starting_joint_angles['left_w2']]

    pnp = PickAndPlace(limb, hover_distance)
    pnp.move_to_start(starting_joint_angles)
    pose_init = pnp.kin.forward_position_kinematics('positions',joint_init)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.5031,#0.4149590815779,
                             y= 0.850,#0.479649402929,
                             z= -0.061,#-0.020637916180073,
                             w=0.14214286450832011)
 
    # Move to the desired starting angles


    idx = 0
    pose = Pose(position=Point(x=0.6363, y=0.291, z=-0.467),
                    orientation=overhead_orientation)
 
    pnp._recursive_servo_to_pose(pose_init, pose)

    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
