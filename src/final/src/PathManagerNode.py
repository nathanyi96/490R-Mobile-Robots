#!/usr/bin/python
''' Module defining the path managing node for the Racecar System '''
import Queue
import rospy
from std_srvs.srv import Empty, EmptyResponse
from lab2.srv import FollowPath, GeneratePath
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header, Float32
import util
import dwave_networkx as dnx
import networkx as nx
import dimod
import numpy as np
import fitdubins
class PathManagerNode(object):
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
        rospy.init_node("path_manager", anonymous=True)
        pose_topic = rospy.get_param("/path_manager/pose_topic", "/sim_car_pose/pose")
        # waypoints_file = rospy.get_param("/path_manager/waypoints_file", "waypoints.txt")
        self.next_paths = None
        self.headings = None
        
        self.current_pose = None
        

        self.PathCompletedService = rospy.Service("/path_manager/path_complete", Empty, self.path_completed_cb)
        self.RunWaypointsService = rospy.Service("/path_manager/run_waypoints", ReadFile, self.waypoints_received)
        self.rp_waypoints = rospy.Publisher('xx_waypoints', Marker, queue_size=100)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.get_current_pose)
        self.controller = rospy.ServiceProxy("/controller/follow_path", FollowPath)
        self.planner = rospy.ServiceProxy("/planner/generate_path", GeneratePath)
        rospy.spin()

    def run(self, poses):
        # determines order of traversal and orientation at each waypoint
        # while has remaining waypoints:
        #   determines start, goal
        #   evaluate time budget
        #   call service to ROSPlannerFinal and retrieve a new path
        #   push new path to queue
        if self.waypoints is None:
            raise RuntimeError('No waypoints is read.')

        # initialize states
        self.next_paths = Queue.Queue()

        first_flag = True
        for i in range(len(self.waypoints) - 1):
            m = self.make_marker(self.waypoints[i], i, "goalpoint")
            self.rp_waypoints.publish(m)
            start = util.particle_to_pose(self.waypoints[i])
            goal = util.particle_to_pose(self.waypoints[i+1])
            path = self.planner(start, goal).path
            if first_flag:
                first_flag = False
                success = self.controller(path)
                print 'sent first path'
            else:
                self.next_paths.put(path)

    def path_completed_cb(self, req):
        print "path completed"
        print "Waiting for next path..."
        next_path = self.next_paths.get(block=True, timeout=10)
        print "Sending next path..."
        success = self.controller(next_path)
        print "Controller started.", success
        return EmptyResponse()

    def make_marker(self, config, i, point_type):
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = str(config)
        marker.id = i
        marker.type = Marker.CUBE
        marker.pose.position.x = config[0]
        marker.pose.position.y = config[1]
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        if point_type == "waypoint":
            marker.color.b = 1.0
        elif point_type == "goalpoint":
            marker.color.g = 1.0
        else:
            marker.color.g = 1.0
        return marker

    def get_current_pose(self, msg):
        self.current_pose = msg.pose

    def waypoints_received(self, req):
        req_waypoints, scores = _parse_waypoints(req.filename)
        #
        #waypoints = _get_roundtrip_waypoints(, req_waypoints)
        #headings = fitdubins.fit_heading_spline(self.waypoints)
        # headings[0] = fe
        #self.run(np.c_[waypoints, headings])



def _parse_waypoints(filename):
    way_points, points = [], []
    with open("../waypoints/" + filename, 'r') as wf:
        line = wf.readline()
        while line:
            x, y, pts = tuple([float(x) for x in line.split(',')])
            way_points.append((x, y)) # dummies
            points.append(pts)
            line = wf.readline()
    return way_points, points


def _init_pose():
    '''
    try kidnap and click, decide which one to use.
    '''
    pass 


def _get_roundtrip_waypoints(start, waypoints):
    ''' Returns the reordered waypoints in the intended traversing order.
        @params the start xy-position
        @params waypoints A list containing the collection of waypoints.
        @return ordered way points as n x 2 ndarray
        Note: The first and last ordered way points are identical.
    '''
    # waypoints_num = len(waypoints)
    # G = nx.complete_graph(waypoints_num)
    # edges = set()
    # for i in range(waypoints_num):
    #     for j in range(i+1, waypoints_num):
    #         dist = np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[j]))
    #         edges.add((i, j, int(dist)))
    # G.add_weighted_edges_from(edges)
    # print dnx.traveling_salesman(G, dimod.ExactSolver())
    ###
    # Try greedy based on the euclidean distance, also add start to the waypoints list
    ###
    tovisit = set(range(len(waypoints)))
    ordered_waypoints = [start]
    while len(tovisit) > 0:
        curr = ordered_waypoints[-1]
        dists = []
        for i in tovisit:
            x, y = waypoints[i]
            dists.append((np.sqrt((x-curr[0])**2 + (y-curr[1])**2), i))
        nxt_point = min(dists)
        ordered_waypoints.append(waypoints[nxt_point[1]])
        tovisit.remove(nxt_point[1])
    ordered_waypoints.append(start)
    return np.array(ordered_waypoints)


def _get_orientation(waypoints):
    return [(waypoint[0], waypoint[1], 0) for waypoint in waypoints]

if __name__ == '__main__':
    path_manager = PathManagerNode()

