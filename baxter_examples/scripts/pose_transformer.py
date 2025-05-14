#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class PoseTransformer:
    def __init__(self):
        rospy.init_node("pose_transformer")

        self.z_offset = 0.133
        self.x_offset = 0
        self.y_offset = 0
        print("trial")
        self.pose_subscriber = rospy.Subscriber('/robot/limb/left/endpoint_state', PoseStamped, self.pose_callback)
        #self.pose_publisher = rospy.Publisher('/robot/left/transformed_pose', PoseStamped, queue_size = 10)

    
    def pose_callback(self, msg):
        transformed_pose = PoseStamped()
        transformed_pose.header = msg.header
        transformed_pose.pose.position.x = msg.pose.position.x + self.x_offset
        transformed_pose.pose.position.y = msg.pose.position.y + self.y_offset
        transformed_pose.pose.position.z = msg.pose.position.z + self.z_offset


        transformed_pose.pose.orientation = msg.pose.orientation
        print(transformed_pose.pose.orientation)
        #self.pose_publisher.publish(transformed_pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        transformer = PoseTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass
