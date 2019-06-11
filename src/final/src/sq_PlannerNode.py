import numpy as np
import matplotlib.pyplot as plt
import util
import dubins
import fitdubins
import rospy
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker
from lab2.msg import XYHV, XYHVPath
from lab2.srv import ReadFile, FollowPath, SignalPathComplete, StampedFollowPath
import sys
sys.path.insert(0, '../../lab3/src')
from DubinsMapEnvironment import DubinsMapEnvironment

class PlannerNode(object):

    def __init__(self, mainpoints_file):
        rospy.init_node('planner', anonymous=True)
        # parse main way points
        self.CURVATURE = 0.075
        self.TRY_DISTANCE = 100
        self.SAMPLED_HEADINGS = np.linspace(0, 2*np.pi, 8, endpoint=False)
        self.SPEED = 0.5

        self.main_path = _parse_mainpoints(mainpoints_file)
        self.main_XYHV = toXYHVPath(self.main_path, desired_speed=self.SPEED, type='main')
        self.waypoints = None
        self.rem_points = None
        pose_topic = rospy.get_param("/path_manager/pose_topic", "/sim_car_pose/pose")
        self.curpose = None

        self.map = self.get_map()
        self.map_x = self.map.info.origin.position.x
        self.map_y = self.map.info.origin.position.y
        self.map_angle = util.rosquaternion_to_angle(self.map.info.origin.orientation)
        self.map_c = np.cos(self.map_angle)
        self.map_s = np.sin(self.map_angle)
        self.map_data = self.load_permissible_region(self.map)

        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)
        self.rp_waypoints = rospy.Publisher('xx_waypoints', Marker, queue_size=20)

        self.planning_env = DubinsMapEnvironment(self.map_data.transpose(), curvature=self.CURVATURE)
        rospy.wait_for_message(pose_topic, PoseStamped)
        self.PathCompletedService = rospy.Service("/path_manager/path_complete", SignalPathComplete, self.path_completed_cb)
        self.RunWaypointsService = rospy.Service("/path_manager/run_waypoints", ReadFile, self.waypoints_received)
        self.controller = rospy.ServiceProxy("/controller/follow_path", StampedFollowPath)
        self.branch_path_complete = False
        # self.rp_waypoints = rospy.Publisher('xx_waypoints', Marker, queue_size=100)
        rospy.spin()

    def pose_cb(self, msg):
        pose = util.rospose_to_posetup(msg.pose)
        self.curpose = (util.world_to_map(np.array([pose]), self.map.info)).reshape(3)

    def waypoints_received(self, req):
        self.waypoints = _parse_waypoints(req.filename)
        self.rem_points = set(range(len(self.waypoints)))
        self.run()

    def run(self):

        ros_poses = []
        for i in range(len(self.waypoints)):
            rpose = util.particle_to_pose(self.waypoints[i])
            ros_poses.append(rpose)
            marker = make_marker(rpose, i, "goalpoint")
            self.rp_waypoints.publish(marker)
        assert len(ros_poses) == len(self.waypoints)
        self.waypoints = util.world_to_map(self.waypoints, self.map.info)
        resp = self.controller(self.main_XYHV)
        print 'Main Trajectory sent.', resp.success
        rate = rospy.Rate(5)
        while len(self.rem_points) > 0:
            curpose = self.curpose
            for wid in self.rem_points:
                if np.linalg.norm(curpose[:2] - self.waypoints[wid, :2]) > self.TRY_DISTANCE:
                    continue
                dubins_path, length = self.try_dubins(curpose, self.waypoints[wid])
                if length > 0:
                    self.hit_waypoints(dubins_path)
                    self.rem_points.remove(wid)
                    break
            rate.sleep()

    

    def try_dubins(self, config1, config2):
        successes = []
        for i in range(self.SAMPLED_HEADINGS.shape[0]):
            config2[2] = self.SAMPLED_HEADINGS[i]
            no_col, dist = self.planning_env.edge_validity_checker(config1, config2)
            if no_col:
                successes.append((dist, i))
        if len(successes) == 0:
            return None, -1
        else:
            config2[2] = self.SAMPLED_HEADINGS[min(successes)[1]]
            return self.planning_env.generate_path(config1, config2)


    def hit_waypoints(self, dubins_path):
        w_dubins_path = util.map_to_world(dubins_path, self.map.info)
        h = Header()
        h.stamp = rospy.Time.now()

        # waypoints = np.concatenate((w_dubins_path, w_dubins_path[::-1]), axis=0)
        # speeds = np.empty(2*len(dubins_path))
        # speeds[:len(dubins_path)] = self.SPEED
        # speeds[len(dubins_path):] = -self.SPEED
        # path = XYHVPath(h,[XYHV(*[waypoint[0], waypoint[1], waypoint[2], speed]) \
        #      for waypoint, speed in zip(waypoints, speeds)])
        waypoints = w_dubins_path
        forward_path = XYHVPath(h,[XYHV(*[waypoint[0], waypoint[1], waypoint[2], speed]) \
             for waypoint, speed in zip(waypoints, speeds)])
        backward_path = XYHVPath(h,[XYHV(*[waypoint[0], waypoint[1], waypoint[2], -speed]) \
             for waypoint, speed in zip(waypoints[::-1], speeds)])
        print "Found path to a waypoint."
        success = self.controller(forward_path)
        print "Controller started.", success

    def path_completed_cb(self, req):
        if req.type == 'branch':
            self.branch_path_complete = True
        print "Completed hitting waypoint."
        print "Back to main trajectory ..."
        success = self.controller(self.main_XYHV)
        print "Controller started.", success
        return EmptyResponse()

    def get_map(self):
        '''
        get_map is a utility function which fetches a map from the map_server
        output:
            map_msg - a GetMap message returned by the mapserver
        '''
        srv_name = "/obs/static_map" #self.params.get_str("static_map", default="/static_map")
        #self.logger.debug("Waiting for map service")
        print "Waiting for map service"
        rospy.wait_for_service(srv_name)
        print "Map service started"
        #self.logger.debug("Map service started")

        map_msg = rospy.ServiceProxy(srv_name, GetMap)().map
        return map_msg

    def load_permissible_region(self, map):
        # TODO: set up caching
        '''
        load_permissible_region uses map data to compute a 'permissible_region'
            matrix given the map information. In this matrix, 0 is permissible,
            1 is not.
        input:
            map - GetMap message
        output:
            pr - permissible region matrix
        '''
        map_data = np.array(map.data)
        array_255 = map_data.reshape((map.info.height, map.info.width))
        pr = np.zeros_like(array_255, dtype=np.uint8)

        # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        pr[array_255 == 0] = 1
        pr = np.logical_not(pr)  # 0 is permissible, 1 is not

        return pr

def _parse_mainpoints(filename):
    points = []
    with open("../waypoints/" + filename, 'r') as wf:
        line = wf.readline()
        while line:
            points.append(tuple([float(x) for x in line.strip().split(' ')]))
            line = wf.readline()
    points.append(points[0])
    mainpoints = np.array(points)
    plt.subplot(111)
    ax = plt.gca()
    headings = fitdubins.fit_heading_spline(mainpoints, ax)
    poses = fitdubins.visualize_dubins(mainpoints, headings, np.tan(0.34)/0.3, ax)
    ax.plot(mainpoints[:, 0], mainpoints[:, 1], 'o')
    ax.set_aspect('equal')
    ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
    plt.savefig('../plots/main_traj.png')
    # plt.show()
    return poses


def _parse_waypoints(filename):
    waypoints = []
    with open("../waypoints/" + filename, 'r') as wf:
        line = wf.readline()
        while line:
            wp = tuple([float(x) for x in line.split(',')])
            waypoints.append(wp)
            line = wf.readline()
    return np.array(waypoints)

def toXYHVPath(waypoints, desired_speed=1.0, type='main'):
    h = Header()
    h.stamp = rospy.Time.now()

    speeds = np.zeros(len(waypoints))
    speeds[:] = desired_speed
    speeds[-1] = 0.0
    path = XYHVPath(h,[XYHV(*[waypoint[0], waypoint[1], waypoint[2], speed]) \
            for waypoint, speed in zip(waypoints, speeds)])
    return path

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
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    if point_type == "waypoint":
        marker.color.b = 1.0
    elif point_type == "goalpoint":
        marker.color.g = 1.0
    else:
        marker.color.g = 1.0
    return marker



if __name__ == '__main__':
    planner = PlannerNode('../waypoints/main.txt')



