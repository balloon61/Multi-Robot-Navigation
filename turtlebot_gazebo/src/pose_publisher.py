#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

from pyquaternion import Quaternion
import numpy as np

position = [(0.0, 0.0, 0.0)]
orientation = [0.0]
x, y, theta = 0, 0, 0
v, w = 0, 0
Ground_truth = []
odom_filter = []

distance_versus_time = []
rospy.init_node('sensor_publisher')

last_time = rospy.get_rostime()
initial_time = 0.0
seq = 0
frame_id = 'map'

less_update = rospy.Time.from_sec(0)

current_time = rospy.Time.now()
last_time2 = rospy.Time.now()
T = Twist()


def __yaw_to_quat(yaw):
    """
      Computing corresponding quaternion q to angle yaw [rad]
      :param yaw
      :return: q
      """
    q = Quaternion(axis=[0, 0, 1], angle=yaw)
    return q.elements


# Subscribe a twist topic publish by move base, and publish the TwistWithCovarianceStamped,
# because ekf localization node only accept TwistWithCovarianceStamped.
def callback_twist(twist_data):
    global v, w, T
    v = twist_data.linear.x
    w = twist_data.angular.z
    T = twist_data
    __pub_twist_cov(twist_data.linear.x, twist_data.angular.z)

# Publish TwistWithCovarianceStamped, basically it just add the stamp, seq, covariance, and frame_id
def __pub_twist_cov(v, w):
    global seq
    seq = seq + 1
    __pubTwist = rospy.Publisher('/twist0', TwistWithCovarianceStamped, queue_size=1)
    twist_cov = TwistWithCovarianceStamped()
    twist_cov.header.stamp = rospy.get_rostime()
    # Not sure if the header frame_id map is correct
    twist_cov.header.frame_id = 'map'
    # twist_cov.child_frame_id = 'odom'
    twist_cov.header.seq = seq
    twist_cov.twist.twist.linear.x = v
    # twist_cov.twist.twist.linear.y = float_list[3] * knots * math.sin(latest_yaw)
    twist_cov.twist.twist.angular.z = w
    ##angular data not used here
    twist_cov.twist.covariance = [0.0001, 0, 0, 0, 0, 0,
                                  0, 0.0001, 0, 0, 0, 0,
                                  0, 0, 0, 0.0000, 0, 0,
                                  0, 0, 0, 0.0000, 0, 0,
                                  0, 0, 0, 0, 0.0000, 0,
                                  0, 0, 0, 0, 0, 0.0001]

    __pubTwist.publish(twist_cov)

# Record ground truth data
def callback_Ground_Truth(data):
    global last_time
    bot_x = data.pose[-1].position.x
    bot_y = data.pose[-1].position.y
    orien = data.pose[-1].orientation
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    position[0] = (bot_x, bot_y, yaw)

    if rospy.get_rostime() > last_time + rospy.Duration.from_sec(0.5):
        last_time = rospy.get_rostime()
        __pub_position(bot_x, bot_y, yaw)

# Publish PoseWithCovarianceStamped, This function is always running but only effect the result
# when the ekf localization node is subscribing pose
def __pub_position(x, y, theta):
    """
    Publishing new initial position (x, y, theta) --> for localization
    :param x x-position of the robot
    :param y y-position of the robot
    :param theta theta-position of the robot
    """
    global Ground_truth

    __pose_pub = rospy.Publisher('/gazebo_pose', PoseWithCovarianceStamped, queue_size=1)

    # add noise to the ground truth
    mu, sigma = 0, 0.15  # mean and standard deviation
    noise = np.random.normal(mu, sigma, 3)
    p = PoseWithCovarianceStamped()
    p.header.stamp = rospy.Time.now()  # + rospy.Duration.from_sec(1.)

    Ground_truth.append([x, y, theta, rospy.get_time() - initial_time])
    p.header.frame_id = 'map'
    p.pose.pose.position.x = x + noise[0]
    p.pose.pose.position.y = y + noise[1]
    quaternion = __yaw_to_quat(theta + noise[2])

    p.pose.pose.orientation.w = quaternion[0]
    p.pose.pose.orientation.x = quaternion[1]
    p.pose.pose.orientation.y = quaternion[2]
    p.pose.pose.orientation.z = quaternion[3]
    print(f"pose = {x:.3f}, {y:.3f}, {np.degrees(theta):.1f}")
    __pose_pub.publish(p)


# Record odom data, published by ekf localization node
def callback_odom_filter(odom_filter_data):
    global odom_filter
    _, _, odom_yaw = euler_from_quaternion(
        [odom_filter_data.pose.pose.orientation.x, odom_filter_data.pose.pose.orientation.y,
         odom_filter_data.pose.pose.orientation.z, odom_filter_data.pose.pose.orientation.w])

    odom_filter.append([odom_filter_data.pose.pose.position.x, odom_filter_data.pose.pose.position.y, odom_yaw,
                        rospy.get_time() - initial_time])


def callback_odom(odomdata):
    __odom_pub = rospy.Publisher('/odom0', Odometry, queue_size=1)
    current_time = rospy.Time.now()
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    # set the position
    odom.pose.pose = odomdata.pose.pose
    odom.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

    odom.child_frame_id = "base_footprint"
    # set the velocity
    odom.twist.twist = odomdata.twist.twist

    # publish the message
    __odom_pub.publish(odom)
    last_time = current_time

# This two function merge list and intersection are used for plotting figure.
# because the two list ground truth and odom filter are not in the same time stamp, and freq,
def merge_list(x1, t1, x2, t2):
    Time = sorted(t1 + t2)
    result1 = []
    result2 = []
    result_time = []
    for t in Time:
        i1, i2 = 1, 1

        if t in t1:

            result1.append(x1[t1.index(t)])
            result_time.append(t)
        else:
            while t1[i1] < t:
                i1 = i1 + 1
                if i1 == len(t1) - 1 or i2 == len(t2) - 1:
                    break
            result1.append(intersection(t1[i1 - 1], t1[i1], t, x1[i1 - 1], x1[i1]))
            result_time.append(t)
        if t in t2:

            result2.append(x2[t2.index(t)])

        else:
            while t2[i2] < t:
                i2 = i2 + 1
                if i1 == len(t1) - 1 or i2 == len(t2) - 1:
                    break
            result2.append(intersection(t2[i2 - 1], t2[i2], t, x2[i2 - 1], x2[i2]))

    return result1, result2, result_time


def intersection(t1, t2, t_mid, v1, v2):
    print(t1, t2, t_mid, v1, v2)
    if t2 == t1:
        return v1
    return (t_mid - t1) * (v2 - v1) / (t2 - t1) + v1


if __name__ == "__main__":
    initial_time = rospy.get_time()

    try:
        # Subscribe for ground truth, this callback also create the ground truth list which is used for the plot
        sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, callback_Ground_Truth)

        # This callback subscribe odom publish by ekf localization node
        sub_odom_filter = rospy.Subscriber('/odometry/filtered_map', Odometry, callback_odom_filter)

        sub_odom_fake = rospy.Subscriber('/odom', Odometry, callback_odom)
        sub_twist = rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, callback_twist)
        rospy.spin()

    finally:
        # Plot the Ground truth and the estimated pose
        plt.scatter([data[0] for data in odom_filter], [data[1] for data in odom_filter], label="Corrected Pose")
        plt.scatter([data2[0] for data2 in Ground_truth], [data2[1] for data2 in Ground_truth], label="Ground Truth")
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()
        # plt.xlim([-2, 11])
        # plt.ylim([-2, 11])
        plt.axis('equal')
        plt.title('x-y')

        # Merge two lists which have different
        odom_int_x, Gtruth_int_x, r_time = merge_list([data[0] for data in odom_filter],
                                                      [data[3] for data in odom_filter],
                                                      [data2[0] for data2 in Ground_truth],
                                                      [data2[3] for data2 in Ground_truth])
        odom_int_y, Gtruth_int_y, _ = merge_list([data[1] for data in odom_filter],
                                                 [data[3] for data in odom_filter],
                                                 [data2[1] for data2 in Ground_truth],
                                                 [data2[3] for data2 in Ground_truth])
        plt.title(label='Ground Truth and EKF Pose with High Sensor Noise\n(standard deviation = 0.15)\nx-y',
                  loc='center')
        plt.show()


        offset = r_time[0]
        for i in range(len(r_time)):
            if i == 0:
                distance_versus_time.append(0)
            else:
                distance_versus_time.append(
                    np.sqrt((Gtruth_int_x[i] - odom_int_x[i]) * (Gtruth_int_x[i] - odom_int_x[i]) +
                            (Gtruth_int_y[i] - odom_int_y[i]) * (Gtruth_int_y[i] - odom_int_y[i])))
            r_time[i] = r_time[i] - offset

        print(distance_versus_time)
        # plt.savefig('0.05.png')
        plt.plot(r_time, distance_versus_time)
        plt.xlabel('t (sec)')
        plt.ylabel('distance (m)')
        plt.xlim([0, 50])
        plt.ylim([0, 2.0])

        plt.title(
            label='Ground Truth and EKF Pose with High Sensor Noise\n(standard deviation = 0.15)\nDistance vs Time',
            loc='center')
        # plt.suptitle(t='Distance vs Time', verticalalignment='bottom')
        plt.show()

        # For some unknown reason this ros.sleep function is not working,
        rospy.sleep(40)

"""
def tf_callback(data):
    if data.transforms[0].header.frame_id == 'map':
        print("in time stamp call back")
        data.transforms[0].header.stamp = data.transforms[0].header.stamp + rospy.Duration.from_sec(0.5)
        
def __publish_fake_odom():
    global x, y, theta, last_time2, v, w, T
    slip_right, slip_left = 0.8, 0.72
    __odom_pub = rospy.Publisher('/odom0', Odometry, queue_size=1)
    current_time = rospy.Time.now()
    dt = (current_time - last_time2).to_sec()
    delta_x = (v * (slip_left + slip_right) / 2 * np.cos(theta) - 0 * np.sin(theta)) * dt
    delta_y = (v * (slip_left + slip_right) / 2 * np.sin(theta) + 0 * np.cos(theta)) * dt
    delta_th = w * dt

    x += delta_x
    y += delta_y
    theta += delta_th
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    # set the position
    quat = quaternion_from_euler(0, 0, theta)
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*quat))

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(v * (slip_left + slip_right) / 2, 0, 0), Vector3(0, 0, w))

    # odom.twist.twist.angular.z  = odom.twist.twist.angular.w * 0.5

    # publish the message
    __odom_pub.publish(odom)
    last_time2 = current_time

"""
