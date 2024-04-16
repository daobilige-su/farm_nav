#! /usr/bin/env python

import numpy as np
import math
# from __future__ import print_function

from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
import rospy
import sys
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float32MultiArray
from tf import transformations
import tf


class GlobalPlanPublisher:
    def __init__(self):
        # param
        self.pub_rate = 1

        # var to store the global plan
        self.global_plan = np.zeros((1, 1))

        # publishers & subscribers
        self.path_pub = rospy.Publisher('/global_plan', Path, queue_size=2)

    def publish_plan(self, global_path):
        pass

def main(args):
    rospy.init_node('global_plan_publisher_node', anonymous=True)
    gl_plan_pub = GlobalPlanPublisher()
    rate = rospy.Rate(gl_plan_pub.pub_rate)
    try:
        while not rospy.is_shutdown():
            gl_plan_pub.publish_plan()
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)