#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
import tf.transformations

rospy.init_node('broadcaster')


def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now() + rospy.Duration.from_sec(0.5),
                     "odom",
                     "map")


def listener():
    rospy.Subscriber("/odometry/filtered_map", Odometry, callback)


    rospy.spin()

if __name__ == '__main__':
    listener()