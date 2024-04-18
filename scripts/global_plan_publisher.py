#! /usr/bin/env python

import numpy as np
import math
# from __future__ import print_function

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion, Twist
import rospy
import sys
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32MultiArray
from tf import transformations
import tf
from transform_tools import *

class GlobalPlanPublisher:
    def __init__(self):
        # param
        self.pub_rate = 1

        # var to store the global plan
        self.global_plan_ypr = np.array([[0, 0, 0, 0, 0, 0],
                                         [0.1, 0, 0, 0, 0, 0],
                                         [0.2, 0, 0, 0, 0, 0],
                                         [0.3, 0, 0, 0, 0, 0],
                                         [0.4, 0, 0, 0, 0, 0],
                                         [0.5, 0, 0, 0, 0, 0],
                                         [0.6, 0, 0, 0, 0, 0],
                                         [0.7, 0, 0, 0, 0, 0],
                                         [0.8, 0, 0, 0, 0, 0],
                                         [0.9, 0, 0, 0, 0, 0],
                                         [1, 0, 0, 0, 0, 0],
                                         [1, 0.1, 0, np.pi/2, 0, 0],
                                         [1, 0.2, 0, np.pi / 2, 0, 0],
                                         [1, 0.3, 0, np.pi / 2, 0, 0],
                                         [1, 0.4, 0, np.pi / 2, 0, 0],
                                         [1, 0.5, 0, np.pi / 2, 0, 0],
                                         [1, 0.6, 0, np.pi / 2, 0, 0],
                                         [1, 0.7, 0, np.pi / 2, 0, 0],
                                         [1, 0.8, 0, np.pi / 2, 0, 0],
                                         [1, 0.9, 0, np.pi / 2, 0, 0],
                                         [1, 1, 0, np.pi/2, 0, 0]])

        self.global_plan_quat = self.global_plan_ypr_to_quat(self.global_plan_ypr)

        # publishers & subscribers
        self.path_pub = rospy.Publisher('/global_plan', Path, queue_size=2)

    def publish_plan(self, global_plan_quat):
        path_msg = Path()
        path_msg.header.stamp = rospy.get_rostime()
        path_msg.header.frame_id = 'map'
        pose_num = global_plan_quat.shape[0]

        for n in range(pose_num):
            # change plan matrix to path msg
            p = PoseStamped()
            p.header.stamp = path_msg.header.stamp
            p.header.frame_id = 'map'

            p.header.seq = n

            trans_quat = global_plan_quat[n, :]
            p.pose.position.x = trans_quat[0]
            p.pose.position.y = trans_quat[1]
            p.pose.position.z = trans_quat[2]
            p.pose.orientation.x = trans_quat[3]
            p.pose.orientation.y = trans_quat[4]
            p.pose.orientation.z = trans_quat[5]
            p.pose.orientation.w = trans_quat[6]

            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)



    def global_plan_ypr_to_quat(self, global_plan_ypr):
        pose_num = global_plan_ypr.shape[0]
        global_plan_quat = np.zeros((pose_num, 7))
        for n in range(pose_num):
            trans_ypr = global_plan_ypr[n, :]
            trans = trans_ypr[0:3]
            ypr = trans_ypr[3:6]
            quat = ypr2quat(ypr).reshape((-1,))
            trans_quat = np.block([trans, quat])
            global_plan_quat[n, :] = trans_quat.copy()

        return global_plan_quat

def main(args):
    rospy.init_node('global_plan_publisher_node', anonymous=True)
    gl_plan_pub_obj = GlobalPlanPublisher()
    rate = rospy.Rate(gl_plan_pub_obj.pub_rate)
    try:
        while not rospy.is_shutdown():
            gl_plan_pub_obj.publish_plan(gl_plan_pub_obj.global_plan_quat)
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)