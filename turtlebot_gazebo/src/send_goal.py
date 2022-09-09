#!/usr/bin/env python

import numpy as np

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pyquaternion import Quaternion
from tqdm import tqdm
import argparse
rospy.init_node('Send_goal', anonymous=True)


def __yaw_to_quat(yaw):
    """
      Computing corresponding quaternion q to angle yaw [rad]
      :param yaw
      :return: q
      """
    q = Quaternion(axis=[0, 0, 1], angle=yaw)
    return q.elements


def movebase_client(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    quaternion = __yaw_to_quat(yaw)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    goal.target_pose.pose.orientation.w = quaternion[0]
    goal.target_pose.pose.orientation.x = quaternion[1]
    goal.target_pose.pose.orientation.y = quaternion[2]
    goal.target_pose.pose.orientation.z = quaternion[3]
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-x','--x', nargs='+', type=float, required=True)
    parser.add_argument('-y','--y', nargs='+', type=float, required=True)
    parser.add_argument('-yaw','--yaw', nargs='+', type=float, required=True)

    args = parser.parse_args()

    movebase_client(args.x[0], args.y[0], args.yaw[0])
    rospy.spin()
