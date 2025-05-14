#!/usr/bin/env python
## This code will subscribe to optitrack body topics transform them to be from the baxter base and republish them

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix

pub_left = rospy.Publisher('/opti/left_arm/transformed_pose', PoseStamped, queue_size = 10)
pub_right = rospy.Publisher('/opti/right_arm/transformed_pose', PoseStamped, queue_size = 10)

g_bo = np.asarray([[0.0279, 0.9995, -0.017, -2.0451],
                 [-0.9991, -0.0273, -0.0327, 1.4312], 
                 [-0.0323, -0.0179, 0.9993, -0.9337],
                 [0, 0, 0, 1]])
g_em_left = np.asarray([[0.0421, -0.0213, -0.9989, -0.0152],
                [-0.9988, -0.0253, -0.0426, 0.0064],
                [0.0262, 0.9995, -0.0203, 0.0496],
                [0, 0, 0, 1]])

g_em_right = np.asarray([[0.0411, -0.0307, -0.9987,-0.0155],
                        [-0.9989, 0.0215, -0.0418, -0.0044],
                        [0.0227, 0.9993, -0.0299, 0.0560],
                        [0, 0, 0, 1]])
print(g_em_right.shape)

g_em_left_inv = np.linalg.inv(g_em_left)
g_em_right_inv = np.linalg.inv(g_em_right)
rospy.init_node('transform_opti')
rate = rospy.Rate(10)

def callback_left(data):
    # try:
    pos = data.pose.position
    quat = data.pose.orientation ## xyzw order
    T_in = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    T_in[0:3,3] = [pos.x, pos.y, pos.z]
    T_out = np.dot(g_bo,T_in)
    T_out = np.dot(T_out,g_em_left_inv)

    new_pose = PoseStamped()
    new_pose.header = data.header
    new_pose.header.frame_id = "transformed_left_opti"

    new_pose.pose.position.x = T_out[0,3]
    new_pose.pose.position.y = T_out[1,3]
    new_pose.pose.position.z = T_out[2,3]

    q = quaternion_from_matrix(T_out)
    new_pose.pose.orientation.x = q[0]
    new_pose.pose.orientation.y = q[1]
    new_pose.pose.orientation.z = q[2]
    new_pose.pose.orientation.w = q[3]

    pub_left.publish(new_pose)

def callback_right(data):
    # try:
    pos = data.pose.position
    quat = data.pose.orientation ## xyzw order
    T_in = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    T_in[0:3,3] = [pos.x, pos.y, pos.z]
    T_out = np.dot(g_bo,T_in)
    T_out = np.dot(T_out,g_em_left_inv)

    new_pose = PoseStamped()
    new_pose.header = data.header
    new_pose.header.frame_id = "transformed_right_opti"

    new_pose.pose.position.x = T_out[0,3]
    new_pose.pose.position.y = T_out[1,3]
    new_pose.pose.position.z = T_out[2,3]

    q = quaternion_from_matrix(T_out)
    new_pose.pose.orientation.x = q[0]
    new_pose.pose.orientation.y = q[1]
    new_pose.pose.orientation.z = q[2]
    new_pose.pose.orientation.w = q[3]

    pub_right.publish(new_pose)

def publish_transformed_optitrack():
    rospy.Subscriber("/mocap_node/left_arm/pose",PoseStamped, callback_left)
    rospy.Subscriber("/mocap_node/right_arm/pose", PoseStamped, callback_right)
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_transformed_optitrack()
    except rospy.ROSInteruptException:
        pass
