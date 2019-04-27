#!/usr/bin/env python

import rospy 
import numpy as np
import time
import utils as Utils
import tf.transformations
import tf
from threading import Lock

from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid

import math
import Queue
from ReSample import ReSampler
from SensorModel import SensorModel
from MotionModel import  KinematicMotionModel
from GlobalLocSensorModel import GlobalLocSensorModel

MAP_TOPIC = "/map"
PUBLISH_PREFIX = '/pf/viz'
PUBLISH_TF = True

'''
  Implements particle filtering for estimating the state of the robot car
'''
# Suggested main 
if __name__ == '__main__':
  rospy.init_node("", anonymous=True) # Initialize the node
  while not rospy.is_shutdown():  # Keep going until we kill it
    # Callbacks are running in separate threads
    try:
        # publish to sim car pose and check the sensor model statistics
        pass


