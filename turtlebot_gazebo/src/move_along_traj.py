#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from gazebo_msgs.msg import ModelStates
import numpy as np
from scipy.spatial.transform import Rotation as R
from threading import Condition

send_another_pred_pose = False
position = [(0.0, 0.0, 0.0)]
p = PoseWithCovarianceStamped()


def __yaw_to_quat(yaw):
    """
      Computing corresponding quaternion q to angle yaw [rad]
      :param yaw
      :return: q
      """
    q = Quaternion(axis=[0, 0, 1], angle=yaw)
    return q.elements


def callback_Ground_Truth(data):
    global last_time, p
    idx = data.name.index("mobile_base")

    bot_x = data.pose[idx].position.x
    bot_y = data.pose[idx].position.y
    orien = data.pose[idx].orientation
    _, _, yaw = R.from_quat([orien.x, orien.y, orien.z, orien.w]).as_rotvec()
    position[0] = (bot_x, bot_y, yaw)

    p.header.stamp = rospy.Time.now()  # + rospy.Duration.from_sec(1.)

    # Ground_truth.append([x, y, theta, rospy.get_time() - initial_time])
    p.header.frame_id = 'map'
    p.pose.pose.position.x = bot_x
    p.pose.pose.position.y = bot_y

    p.pose.pose.orientation = orien


# def __pub_position(x, y, theta):
#     """
#     Publishing new initial position (x, y, theta) --> for localization
#     :param x x-position of the robot
#     :param y y-position of the robot
#     :param theta theta-position of the robot
#     """
#     global Ground_truth


#     # add noise to the ground truth
#     mu, sigma = 0, 0.15  # mean and standard deviation
#     noise = np.random.normal(mu, sigma, 3)
#     p = PoseWithCovarianceStamped()
#     p.header.stamp = rospy.Time.now()  # + rospy.Duration.from_sec(1.)

#    # Ground_truth.append([x, y, theta, rospy.get_time() - initial_time])
#     p.header.frame_id = 'map'
#     p.pose.pose.position.x = x
#     p.pose.pose.position.y = y
#     quaternion = __yaw_to_quat(theta)

#     p.pose.pose.orientation.w = quaternion[0]
#     p.pose.pose.orientation.x = quaternion[1]
#     p.pose.pose.orientation.y = quaternion[2]
#     p.pose.pose.orientation.z = quaternion[3]
#     print(f"pose = {x:.3f}, {y:.3f}, {np.degrees(theta):.1f}")
#     __pose_pub.publish(p)


def pose_update_callback(pose_update_msg):
    # Extract the pose and pose error from the message.
    position = pose_update_msg.pose.pose.position
    x, y, z = position.x, position.y, position.z
    orient = pose_update_msg.pose.pose.orientation
    roll, pitch, yaw = R.from_quat(
        [orient.x, orient.y, orient.z, orient.w]).as_rotvec()

    covariance = np.array(pose_update_msg.pose.covariance)
    output_str = '[ FAKE TARGET ] Received pose update from observer @ ' \
                 + f'time {pose_update_msg.header.stamp}:\n' \
                 + f'- pose [xyz in m] ({x:.2f}, {y:.2f}, {z:.2f})\n' \
                 + f'- orientation [rpy in degs] ({np.degrees(roll):.1f},' \
                 + f'{np.degrees(pitch):.1f}, {np.degrees(yaw):.1f})\n' \
                 + f'- covariance:\n{covariance.reshape(6, 6)}'
    rospy.loginfo(output_str)

    # Allow the main function to send another predicted pose to the observer.
    global send_another_pred_pose
    send_another_pred_pose = True


# end def


def main():
    # Set up the ROS node.
    rospy.init_node('test_fake_target_node', log_level=rospy.DEBUG)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_Ground_Truth)
    __pose_pub = rospy.Publisher('/gazebo_pose', PoseWithCovarianceStamped, queue_size=1)

    marker_position_list = list()
    marker_position_list.append((2.5, 1.3))
    marker_position_list.append((7.6, 5.2))
    marker_position_list.append((-2.5, -5.2))
    pred_pose_topic = '/target/target_future_pose'
    pose_update_topic = '/observer/target_pose_meas'

    pred_pose_pub = rospy.Publisher(pred_pose_topic, PoseWithCovarianceStamped,
                                    queue_size=5)
    pose_update_sub = rospy.Subscriber(pose_update_topic,
                                       PoseWithCovarianceStamped, pose_update_callback)
    PubTwist = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

    cmd = Twist()
    global send_another_pred_pose, p
    for ri in range(8):
        cmd.linear.x = 0.2
        cmd.angular.z = 0

        for _ in range(1000):
            PubTwist.publish(cmd)
            rospy.sleep(0.01)
        global p
        rot = R.from_rotvec([0, 0, np.radians(ri * 25.)]).as_matrix()
        cov = np.eye(6)
        cov[[0, 1], [0, 1]] = [1.5, 3]
        cov[:3, :3] = np.matmul(rot, np.matmul(cov[:3, :3], rot.T))
        p.pose.covariance = cov.flatten().tolist()

        for _ in range(5):
            # PubTwist.publish(cmd)
            pred_pose_pub.publish(p)
            rospy.sleep(0.1)

        # Wait to send another predicted pose to the observer.
        send_another_pred_pose = False
        rospy.loginfo("[ FAKE TARGET ] Waiting for a pose update.")
        while send_another_pred_pose == False:
            rospy.sleep(0.5)
        rospy.loginfo("[ FAKE TARGET ] Sleeping for a few seconds.")

    # end for
    rospy.loginfo('Test complete!')
    rospy.spin()


# end def

if __name__ == "__main__":
    try:
        main()

    except:
        pass