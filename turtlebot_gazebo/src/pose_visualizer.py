#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf
from nav_msgs.msg import Odometry
import tf.transformations
from geometry_msgs.msg import Twist


def callback(msg):
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('observer_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(2) # Hz
    p = PoseStamped()
    p.pose.position = msg.pose.pose.position
    p.pose.orientation = msg.pose.pose.orientation

    p.header.seq = 1
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = 'map'
    pub.publish(p)
    print(msg.pose.pose.position, msg.pose.pose.orientation)


def listener():
    rospy.init_node('broadcaster')
    rospy.Subscriber("/mavros/odometry/in", Odometry, callback)
    publisher(1, 1, 1, 0, 0, 0, 1)
    rospy.spin()


def publisher(x, y, z, orien_x, orien_y, orien_z, orien_w):
    pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        p = PoseStamped()
        p.header.seq = 1
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        # Make sure the quaternion is valid and normalized
        p.pose.orientation.x = orien_x
        p.pose.orientation.y = orien_y
        p.pose.orientation.z = orien_z
        p.pose.orientation.w = orien_w
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy:
        pass

    # !/usr/bin/env python





