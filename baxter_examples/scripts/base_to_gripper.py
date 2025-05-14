#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

class BaseToGripperTransformPublisher:
    def __init__(self):
        rospy.init_node('base_to_gripper_transform_publisher')

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("/base", "left_gripper_base", rospy.Time(), rospy.Duration(4.0))
        self.transform_publisher = rospy.Publisher('fwd_kinematics_left', PoseStamped, queue_size = 10)
    
    def publish_transform(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform("/base", "/left_gripper_base", rospy.Time(0))

                transform_msg = PoseStamped()
                transform_msg.header.stamp = rospy.Time.now()
                transform_msg.header.frame_id = 'base'
                
                transform_msg.pose.position.x = trans[0]
                transform_msg.pose.position.y = trans[1]
                transform_msg.pose.position.z = trans[2]

                transform_msg.pose.orientation.x = rot[0]
                transform_msg.pose.orientation.y = rot[1]
                transform_msg.pose.orientation.z = rot[2]
                transform_msg.pose.orientation.w = rot[3]

                self.transform_publisher.publish(transform_msg)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Transformation not available between /base and /left_gripper_base")
            
            rate.sleep()
    def run(self):
        self.publish_transform()

if __name__ =='__main__':
    try:
        pose_publisher = BaseToGripperTransformPublisher()
        pose_publisher.run()

    except rospy.ROSInterruptException:
        pass