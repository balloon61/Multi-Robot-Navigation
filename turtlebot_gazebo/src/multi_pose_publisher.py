#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3


from tf.transformations import euler_from_quaternion, quaternion_from_euler

from nav_msgs.msg import Odometry

from pyquaternion import Quaternion
import numpy as np


rospy.init_node('multi-robot-goal')


# def __yaw_to_quat(yaw):
#     """
#       Computing corresponding quaternion q to angle yaw [rad]
#       :param yaw
#       :return: q
#       """
#     q = Quaternion(axis=[0, 0, 1], angle=yaw)
#     return q.elements

class robot:
    def __init__(self, number: int):
        self.name = '/tb2_' + str(number)
        self.goal_publisher = rospy.Publisher(self.name + '/move_base_simple/goal', PoseStamped, queue_size=1)
        self.vel_publisher = rospy.Publisher(self.name + '/cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def send_goal(self, x: float, y: float, theta:float):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()  # + rospy.Duration.from_sec(1.)
        p.header.frame_id = 'map'
        p.header.seq = 1
        p.pose.position.x = x
        p.pose.position.y = y
        q = Quaternion(axis=[0, 0, 1], angle=theta)
        quaternion = q.elements

        p.pose.orientation.w = quaternion[0]
        p.pose.orientation.x = quaternion[1]
        p.pose.orientation.y = quaternion[2]
        p.pose.orientation.z = quaternion[3]
        print(f"pose = {x:.3f}, {y:.3f}, {np.degrees(theta):.1f}")
        rospy.sleep(1)

        self.goal_publisher.publish(p)

    def send_vel(self, v:float, w:float):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        print(f"command = {v:.3f}, {w:.3f}")
        self.vel_publisher.publish(t)

class multi_robot_network:
    def __init__(self, number_of_robot: int):
        self.number_of_robot = number_of_robot
        self.robot_network = list()
        for n in range(number_of_robot):
            self.robot_network.append(robot(n))
    def give_moving_action(self, goal_list: list):
        for i in range(self.number_of_robot):
            x, y, theta = goal_list[i]
            self.robot_network[i].send_goal(x, y, theta)
            rospy.sleep(1)

def test_move_function(network: multi_robot_network):
    goal = [(6.5, 5., .5), (5., 6.5, .0), (6.5, 6.5, .0)]
    network.give_moving_action(goal_list=goal)

if __name__ == "__main__":

    # Subscribe for ground truth, this callback also create the ground truth list which is used for the plot
    test_move_function(multi_robot_network(3))
    rospy.spin()




