#!/usr/bin/python
''' Module defining the path managing node for the Racecar System '''
import Queue
import rospy
from std_srvs.srv import Empty, EmptyResponse
from lab2.srv import FollowPath, GeneratePath, ReadFile
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header, Float32
import util
import dwave_networkx as dnx
import networkx as nx
import dimod
import numpy as np
import matplotlib.pyplot as plt
import fitdubins
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class CollisionHandlerNode(object):
    ''' Path-managing ROS node of running Racecar system

        This ROSNode take a set of waypoints and periodically
        sends a path for the controller to follow.

        Input parameter:
            /path_manager/waypoints_file - the text file containing waypoints to traverse

        Subscribed topics:
            /sim_car_pose/pose - current pose of the car
        Published topics: None
        Provided services:
            /plan_manager/run_waypoints (lab3/ReadFile) - run waypoints from a file
            /plan_manager/path_complete (lab2/FollowPath) - notify manager of completing a path
        Called services:
            /controller/follow_path (lab2/FollowPath) - notify controller a path to follow
            /planner/evaluate_path (lab2/FollowPath) - retrieve a path from planner

    '''

    ## Constants

    def __init__(self):
        self.pub_safety = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)
