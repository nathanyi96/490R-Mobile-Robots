import numpy as np
import matplotlib.pyplot as plt
import util
import dubins
import fitdubins
import rospy
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
#import matplotlib.pyplot as plt
import fitdubins
from lab2.msg import XYHVPath, XYHV
import os
import time
import IPython
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
class PlannerNode(object):

    def __init__(self):
        rospy.init_node('planner', anonymous=True)
        # parse main way points
        mainpoints_file = '../waypoints/main.txt'
        self.main_path_poses = _parse_mainpoints(mainpoints_file)
        self.waypoints = []
        self.main_XYHV = toXYHVPath(self.main_path_poses, desired_speed=0.5)
        pose_topic = rospy.get_param("/path_manager/pose_topic", "/sim_car_pose/pose")
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.update_state)
        self.RunWaypointsService = rospy.Service("/path_manager/run_waypoints", ReadFile, self.waypoints_received)
        self.PathCompletedService = rospy.Service("/path_manager/path_complete", Empty, self.path_completed_cb)
        self.controller = rospy.ServiceProxy("/controller/follow_path", FollowPath)
        self.planner = rospy.ServiceProxy("/planner/generate_path", GeneratePath)
        self.rp_waypoints = rospy.Publisher('xx_waypoints', Marker, queue_size=100)
        self.RADIUS = 50 # pixe
        self.next_paths = None
        self.current_pose = np.zeros(3)
        self.main_path = True

        rospy.spin()

    def update_state(self, msg):
        pose = np.array(util.rospose_to_posetup(msg.pose)).reshape([3, ])
        self.current_pose = pose

    def run(self):
        rate = rospy.Rate(10)
        while(self.waypoints.shape[0] > 0):
            cur_pose = np.array([[self.current_pose[0], self.current_pose[1]]])
            dist = np.linalg.norm(cur_pose - np.array(self.waypoints)[:,:2], axis=-1)
            min_idx = np.argmin(dist[1:-2]) + 1 # remove start for traversing
            print 'current dist', self.main_path, dist, self.waypoints.shape[0]
            if dist[min_idx] < self.RADIUS:
                print 'start roundtrip'
                IPython.embed()
                self.run_one_pose(self.waypoints[min_idx], backward=True)
                self.waypoints = np.concatenate((self.waypoints[:min_idx], self.waypoints[min_idx+1:]))
                print 'waypoint left: ', self.waypoints.shape[0]
            elif self.main_path:
                self.main_path = False
                self.controller(self.main_XYHV)
            rate.sleep()
        self.controller(self.main_XYHV) # finish path

    def run_one_pose(self, pose, backward=False):
        if pose is None:
            raise RuntimeError('No waypoints is read.')
        # initialize states
        self.next_paths = Queue.Queue()

        # convert poses to ROS Pose and Marker
        rpose = util.particle_to_pose(pose)
        cur_pose = util.particle_to_pose(self.current_pose) # might be outdated
        marker = make_marker(rpose, 0, "goalpoint")
        self.rp_waypoints.publish(marker)
        first_flag = True
        sstart = 0
        max_cnt = 8
        cnt = 1
        print 'Computing path from key points to waypoint...'
        resp = self.planner(cur_pose, rpose, backward)
        path = resp.path
        while not resp.success and cnt <= max_cnt:
            # perturb heading and keep runing
            perturb_pose = pose
            perturb_pose[2] += cnt / max_cnt * (2 * np.pi)

            rpose = util.particle_to_pose(perturb_pose)
            test_plantime = rospy.get_time()
            resp = self.planner(cur_pose, rpose, backward)
            print rospy.get_time() - test_plantime
            print 'Failed to find path.'
            cnt += 1
        if not resp.success:
            print 'Failed to find path with all sampled headings.'
            return
        print 'Path found.'
        path = resp.path
        if path is not None:
            self.controller(path)
            backward_path = path.waypoints
            backward_path = [[p.x, p.y, p.h, -p.v] for p in backward_path[::-1]]
            backward_path = toXYHVPath(backward_path)
            self.main_path = False
            i = 0
            while not self.main_path:
                i += 1
            print i
            self.controller(backward_path)


    def waypoints_received(self, req):
        plt.subplot(111)
        ax = plt.gca()
        req_waypoints, scores = _parse_waypoints(req.filename)
        curr_pose = self.current_pose
        waypoints = _get_roundtrip_waypoints((curr_pose[0], curr_pose[1]), req_waypoints)
        headings = fitdubins.fit_heading_spline(waypoints, ax)
        headings[0] = curr_pose[2]
        self.waypoints = np.c_[waypoints, headings]
        for i in range(self.waypoints.shape[0]):
            rpose = util.particle_to_pose(self.waypoints[i])
            marker = make_marker(rpose, i, "goalpoint")
            self.rp_waypoints.publish(marker)
        self.run()

    def path_completed_cb(self, req):
        if not self.main_path:
            print 'one path complete'
            print len(self.waypoints)
            self.main_path = True # back to main path
        return EmptyResponse()

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

def make_marker(config, i, point_type):
    marker = Marker()
    marker.header = Header()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "map"
    marker.ns = "waypoints"
    marker.id = i
    marker.type = Marker.ARROW
    marker.pose = config
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1.0
    if point_type == "waypoint":
        marker.color.b = 1.0
    elif point_type == "goalpoint":
        marker.color.g = 1.0
    else:
        marker.color.g = 1.0
    return marker

def _parse_mainpoints(filename):
    points = []
    with open("../waypoints/" + filename, 'r') as wf:
        line = wf.readline()
        while line:
            points.append(tuple([float(x) for x in line.strip().split(' ')]))
            line = wf.readline()
    points.append(points[0])
    mainpoints = np.array(points)
    #:
    #plt.subplot(111)
    ax = plt.gca()
    headings = fitdubins.fit_heading_spline(mainpoints, ax)
    poses = fitdubins.visualize_dubins(mainpoints, headings, np.tan(0.34)/0.3, ax)
    #ax.plot(mainpoints[:, 0], mainpoints[:, 1], 'o')
    #ax.set_aspect('equal')
    #ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
    #plt.savefig('../plots/main_traj.png')
    #plt.show()
    return poses

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

def toXYHVPath(waypoints, desired_speed=1.0):
    h = Header()
    h.stamp = rospy.Time.now()

    speeds = np.zeros(len(waypoints))
    speeds[:] = desired_speed
    speeds[-1] = 0.0
    path = XYHVPath(h,[XYHV(*[waypoint[0], waypoint[1], waypoint[2], speed]) \
            for waypoint, speed in zip(waypoints, speeds)])
    return path



if __name__ == '__main__':
    path_manager = PlannerNode()
