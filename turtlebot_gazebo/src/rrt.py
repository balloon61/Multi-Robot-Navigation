#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import argparse
import sys

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData


def map_callback(map):
    print(len(map.data), map.info)#, len(map.data[0]))



if __name__ == "__main__":

    rospy.init_node("rrtstar")
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, map_callback)

    rospy.spin()
