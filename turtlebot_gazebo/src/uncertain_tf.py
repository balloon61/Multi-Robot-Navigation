#!/usr/bin/env python3

import rospy
import pymrpt
from pymrpt import obs
from pymrpt.obs import *
import pymrpt.poses
from pymrpt.poses import CPose3D, CPose3DPDFGaussian, CPose2D
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovariance, Pose
import numpy as np

p1, p2, p = PoseWithCovariance(), PoseWithCovariance(), PoseWithCovariance()


def uncertainty_transformation(tfs: list, direction: list, PoseWithCovar: PoseWithCovariance) -> PoseWithCovariance:
    # transfer landmark pose to CPose3DPDFGaussian
    P = CPose3DPDFGaussian()  # Pose with covar of the marker
    P_roll, P_pitch, P_yaw = euler_from_quaternion([PoseWithCovar.pose.orientation.x, PoseWithCovar.pose.orientation.y,
                                                    PoseWithCovar.pose.orientation.z, PoseWithCovar.pose.orientation.w])
    P.mean.setFromValues(PoseWithCovar.pose.position.x, PoseWithCovar.pose.position.y, PoseWithCovar.pose.position.z,
                         P_yaw, P_pitch, P_roll)
    P.cov = PoseWithCovar.covariance
    zipped = zip(tfs, direction)
    # transformation from  marker frame {P} -> camera frame {C} -> robot frame {R} -> odom frame {O} -> map frame {M}
    for TF, dir in zipped:
        # Transfer tf to CPose3DPDFGaussian so that it can do the CPose3DPDFGaussian transformation
        Next_pose = CPose3DPDFGaussian()
        Next_pose_roll, Next_pose_pitch, Next_pose_yaw = euler_from_quaternion([TF[1][0], TF[1][1], TF[1][2], TF[1][3]])
        Next_pose.mean.setFromValues(TF[0][0], TF[0][1], TF[0][2], Next_pose_yaw, Next_pose_pitch, Next_pose_roll)

        # Transformation using +/-
        if dir == '+':
            P = P + Next_pose
        elif dir == '-':
            P = P - Next_pose

        print("Current TF result:\n", P)

    # Convert CPose3DPDFGaussian by to PoseWithCovariance
    res = convert_cpose3dpdfgaussian_to_ros_pose_with_covar(P)
    print("result:\n", res)

    return res

def convert_cpose3dpdfgaussian_to_ros_pose_with_covar(Cpos: CPose3DPDFGaussian) -> PoseWithCovariance:
    robot_pose = PoseWithCovariance()
    robot_pose.pose.position.x = Cpos.mean.x
    robot_pose.pose.position.y = Cpos.mean.y
    robot_pose.pose.position.z = Cpos.mean.z
    q = quaternion_from_euler(Cpos.mean[5], Cpos.mean[4], Cpos.mean[3]) # roll, pitch, yaw
    robot_pose.pose.orientation.x = q[0]
    robot_pose.pose.orientation.y = q[1]
    robot_pose.pose.orientation.z = q[2]
    robot_pose.pose.orientation.w = q[3]
    robot_pose.covariance = Cpos.cov

    return robot_pose



# create observation
if __name__ == "__main__":
    rospy.init_node("uncertainty_tf")
#    rospy.Subscriber('/tf',  tf2_msgs/TFMessage, callback)
    direction = ['+', '-', '+']

    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform('/odom', '/base_footprint', rospy.Time(), rospy.Duration(4.0))
    listener.waitForTransform('/base_footprint', '/base_camera', rospy.Time(), rospy.Duration(4.0))

    tf1 = listener.lookupTransform('/map', '/odom', rospy.Time())
    tf2 = listener.lookupTransform('/odom', '/base_footprint', rospy.Time())
    tf3 = listener.lookupTransform('/base_footprint', '/base_camera', rospy.Time())
    print(tf1, tf2, tf3)
    TFs = [tf1, tf2, tf3]

    landmark_pose_cov = PoseWithCovariance()
    landmark_pose_cov.pose.position.x = 0.
    landmark_pose_cov.pose.position.y = 0.
    landmark_pose_cov.pose.position.z = 0.
    q = quaternion_from_euler(0, 0, 0) # roll, pitch, yaw
    landmark_pose_cov.pose.orientation.x = q[0]
    landmark_pose_cov.pose.orientation.y = q[1]
    landmark_pose_cov.pose.orientation.z = q[2]
    landmark_pose_cov.pose.orientation.w = q[3]
    landmark_pose_cov.covariance = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, np.pi/8, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, np.pi/4, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, np.pi/2]

    uncertainty_transformation(TFs, direction, landmark_pose_cov)




    """
    
    
    
def uncertainty_tf(C: Pose, L: PoseWithCovariance, R: Pose):
    # CC = CPose3D(C.position.x, C.position.y, C.position.z, 0.5, 0, 0) # Camera pose
    # CR = CPose3D(R.position.x, R.position.y, 0, 0, 0, 0) # Robot pose
    CR = CPose3DPDFGaussian()  # Robot pose
    CR_roll, CR_pitch, CR_yaw = euler_from_quaternion(
        [R.orientation.x, R.orientation.y, R.orientation.z, R.orientation.w])
    CR.mean.setFromValues(R.position.x, R.position.y, R.position.z, CR_yaw, CR_pitch,
                          CR_roll)  # seems like it is Yaw Pitch Roll

    # print(CR)

    CL = CPose3DPDFGaussian()  # Landmarkpose
    # CL_rot = euler_from_quaternion(L.pose.orientation.x, L.pose.orientation.y, L.pose.orientation.z, L.pose.orientation.w)
    CL_roll, CL_pitch, CL_yaw = euler_from_quaternion(
        [L.pose.orientation.x, L.pose.orientation.y, L.pose.orientation.z, L.pose.orientation.w])
    CL.mean.setFromValues(L.pose.position.x, L.pose.position.y, L.pose.position.z, CL_yaw, CL_pitch, CL_roll)
    CL.cov = L.covariance

    CC = CPose3DPDFGaussian()  # Camera
    CC_roll, CC_pitch, CC_yaw = euler_from_quaternion(
        [C.orientation.x, C.orientation.y, C.orientation.z, C.orientation.w])
    # CC.mean.setFromValues(C.position.x, C.position.y, C.position.z, CC_yaw, CC_pitch, CC_roll)
    CC.mean.setFromValues(C.position.x, C.position.y, C.position.z, np.pi / 2, 0., np.pi / 4)

    # CC.cov = L.covariance

    # print("Robot info:", CR)
    # print("Camera info:", CC)
    print("W info:\n", CC)
    print("P info:\n", CL)
    # print("Landmark info:", CL)
    # transfer from Pose or PoseWithCovariance to CPose2D/3D

    # Transformation
    print(
        "////////////////////////////////////////////////////\n/                  Transformation                  /\n////////////////////////////////////////////////////")
    CL2 = CL - CC  # - CR
    print("L2 info:\n", CL2)
    return CL2

        tfs = list()
        tfs.append(tf1)
        tfs.append(tf2)
        tfs.append(tf3)
        
            tf1 = tf.Transformer(True, rospy.get_rostime())
    print(tf1.transformer)
   # tf1.header.stamp = rospy.get_rostime()
   # tf1.header.frame_id = 'camera_frame'
    tf1.transform.translation.x = 1.
    tf1.transform.translation.y = 2.
    tf1.transform.translation.z = 3.


    tf2 = tf.Transformer(True, rospy.get_rostime())
    tf2.header.stamp = rospy.get_rostime()
    tf2.header.frame_id = 'odom'
    q2 = quaternion_from_euler(np.pi / 2, 0, 0) # roll, pitch, yaw
    tf2.transform.rotation.x = q2[0]
    tf2.transform.rotation.y = q2[1]
    tf2.transform.rotation.z = q2[2]
    tf2.transform.rotation.w = q2[3]

    tf3 = tf.Transformer(True, rospy.get_rostime())
    tf3.header.stamp = rospy.get_rostime()
    tf3.header.frame_id = 'map'
    q3 = quaternion_from_euler(np.pi / 2, 0, 0) # roll, pitch, yaw
    tf3.transform.rotation.x = q3[0]
    tf3.transform.rotation.y = q3[1]
    tf3.transform.rotation.z = q3[2]
    tf3.transform.rotation.w = q3[3]
    """
    """camera_pose = Pose()
    robot_pose = Pose()
    landmark_pose_cov = PoseWithCovariance()

    camera_pose.position.x = 0.
    camera_pose.position.y = 0.
    camera_pose.position.z = 0.
    q = quaternion_from_euler(0, 0, np.pi/2) # roll, pitch, yaw
    camera_pose.orientation.x = q[0]
    camera_pose.orientation.y = q[1]
    camera_pose.orientation.z = q[2]
    camera_pose.orientation.w = q[3]

    robot_pose.position.x = 0.
    robot_pose.position.y = 0.
    robot_pose.position.z = 0.
    q2 = quaternion_from_euler(0, 0, 0) # roll, pitch, yaw
    robot_pose.orientation.x = q2[0]
    robot_pose.orientation.y = q2[1]
    robot_pose.orientation.z = q2[2]
    robot_pose.orientation.w = q2[3]

    landmark_pose_cov.pose = robot_pose
    landmark_pose_cov.pose.position.z = robot_pose.position.z
    landmark_pose_cov.covariance = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, np.pi/8, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, np.pi/4, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, np.pi/2]
    uncertainty_tf(camera_pose, landmark_pose_cov, robot_pose)"""
