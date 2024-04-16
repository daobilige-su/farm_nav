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


class PurePursuitPlannerFws:
    def __init__(self):
        # param
        self.controller_freq = 1

        # var to store the current global plan
        self.global_plan = None

        # tf listener
        self.tf_listener = tf.TransformListener()

        # publishers & subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.cmd_vel_sub = rospy.Subscriber("/global_plan", Path, self.update_global_plan)

    def update_global_plan(self, global_path):
        pass

    def compute_plan(self):
        # skip if the global plan is None
        if self.global_plan is None:
            rospy.loginfo('local_planner_pure_pursuit_fws_node: no global plan')
            return

        # listen to the latest tf for base_link in map
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        #

        return True


def main(args):
    rospy.init_node('local_planner_pure_pursuit_fws_node', anonymous=True)
    planner = PurePursuitPlannerFws()
    rate = rospy.Rate(planner.controller_freq)
    try:
        while not rospy.is_shutdown():
            planner.compute_plan()
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)