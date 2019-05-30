#!/usr/bin/env python

from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetMap

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry

from lab2.msg import XYHV, XYHVPath
from lab2.srv import FollowPath

import numpy as np
from scipy import signal
import rospy
import os
from matplotlib import pyplot as plt
import networkx as nx

from DubinsMapEnvironment import DubinsMapEnvironment
from MapEnvironment import MapEnvironment
from DubinsSampler import DubinsSampler
import dubins_graph_maker as graph_maker
import pickle
import util
import lazy_astar
from Sampler import Sampler
from EllipseSampler import informed_sample
import time
class ROSPlanner:
    def __init__(self, heuristic_func, weight_func, num_vertices, connection_radius,
        graph_file='ros_graph_sparse.pkl', do_shortcut=False, num_goals=1,
        curvature=0.02, plan_time=2, plan_with_budget=False):
        """
        @param heuristic_func: Heuristic function to be used in lazy_astar
        @param weight_func: Weight function to be used in lazy_astar
        @param num_vertices: Number of vertices in the graph
        @param connection_radius: Radius for connecting vertices
        @param graph_file: File to load. If provided and the file does not exist,
        then the graph is constructed and saved in this filename
        @param do_shortcut: If True, shortcut the path
        @param num_goals: If > 1, takes multiple goals

        """

        rospy.init_node('planner', anonymous=True)

        # load map
        self.map = self.get_map()
        self.map_x = self.map.info.origin.position.x
        self.map_y = self.map.info.origin.position.y
        self.map_angle = util.rosquaternion_to_angle(self.map.info.origin.orientation)
        self.map_c = np.cos(self.map_angle)
        self.map_s = np.sin(self.map_angle)
        self.map_data = self.load_permissible_region(self.map)
        self.goals = None
        self.total_planning_time = plan_time
        self.time_left = plan_time
        self.plan_with_budget = plan_with_budget
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)
        rospy.Subscriber('/sim_car_pose/pose', PoseStamped, self.get_current_pose)

        self.multi_goals = num_goals > 1
        self.num_goals = num_goals
        if self.multi_goals:
            print("Accept multiple goals")
            self.goals = []
            self.route_sent = False

        self.do_shortcut = do_shortcut

        # Setup planning env
        self.planning_env = DubinsMapEnvironment(self.map_data.transpose(), curvature=curvature)
        # injected code: update xlimit, ylimit of environment
        # to tightly hold all permissible grids.
        self.tight_sampling_limits(self.planning_env)
        self.sampler = DubinsSampler(self.planning_env)

        if os.path.exists(graph_file):
            print("Opening {}".format(graph_file))
            self.G = graph_maker.load_graph(graph_file)
            #self.planning_env.visualize_graph(self.G, saveto="")
        else:
            print("Generating graph, wait for completion")
            self.G = graph_maker.make_graph(self.planning_env,
                sampler=self.sampler,
                num_vertices=num_vertices,
                connection_radius=connection_radius,
                saveto=graph_file,
                lazy=True)

            print("visualize graph")
            self.planning_env.visualize_graph(self.G, saveto="graph.png")

        self.num_vertices = num_vertices
        self.connection_radius = connection_radius

        self.heuristic_func = lambda n1, n2: heuristic_func(n1, n2, self.planning_env, self.G)
        self.weight_func = lambda n1, n2: weight_func(n1, n2, self.planning_env, self.G)

        print("Ready to take goals")
        self.curvature = curvature

        debug_plan = False
        if debug_plan:
            self.new_goal = False
            self.path_nodes = None
            while not rospy.is_shutdown():
                if self.new_goal:
                    self.planning_env.visualize_plan(self.G, self.path_nodes,
                            tuple(self.start),tuple(self.goal))
                    self.new_goal = False
        rospy.spin()


    def tic_toc(self, last_time):
        self.time_left -= last_time
        print 'time left', self.time_left
        return time.time()

    def plan_to_goal(self, start, goal):
        """
        Plan a path from start to goal
        Return a path
        """
        # Implement here
        # Plan with lazy_astar
        self.time_left= self.total_planning_time 
        
        start, goal = tuple(start), tuple(goal)
        self.G, _ = graph_maker.add_node(self.G, start, env=self.planning_env,
                connection_radius=self.connection_radius)
        self.G, _ = graph_maker.add_node(self.G, goal, env=self.planning_env,
                connection_radius=self.connection_radius)
        rospy.loginfo('planning from start to goal...')
        start_time = time.time()
        path_nodes = lazy_astar.astar_path(self.G, source=start, target=goal,
                weight=self.weight_func, heuristic=self.heuristic_func, return_dist=self.plan_with_budget)
        if self.plan_with_budget:
            dist = path_nodes[1]
            path_nodes = path_nodes[0]
        start_time = self.tic_toc(time.time() - start_time)
        idx = 0 
        max_sample_num = 8
        while self.plan_with_budget and self.time_left > 0:
            added_nodes, ellipse = self.densify_graph(start, goal, dist, max_sample_num / (2 ** idx))
            self.planning_env.visualize_graph(self.G, start, goal, added_nodes, ellipse, 'densify_{}'.format(idx))
            path_nodes, dist = lazy_astar.astar_path(self.G, source=start, target=goal,
            weight=self.weight_func, heuristic=self.heuristic_func, return_dist=True)           
            self.path_nodes = path_nodes
            self.new_goal = True
            if self.do_shortcut:
                path_nodes = self.planning_env.shortcut(self.G, path_nodes)
            path = self.planning_env.get_path_on_graph(self.G, path_nodes)
            start_time = self.tic_toc(time.time() - start_time)

        rospy.loginfo('done planning.')
        self.path_nodes = path_nodes
        self.new_goal = True
        if self.do_shortcut:
            path_nodes = self.planning_env.shortcut(self.G, path_nodes)
        path = self.planning_env.get_path_on_graph(self.G, path_nodes)
        return path

    def densify_graph(self, start_config, goal_config, min_cost, sample_num):
        '''
        sample poses and add to graph if h+g <= cost within time limit.
        '''
        start, end = np.array(start_config), np.array(goal_config)
        distance_min = np.linalg.norm(start - end) 
        distance_max = min_cost
        
        start_time = self.tic_toc(0.0)
        added_nodes = []
        vertices, ellipse = informed_sample(distance_max, distance_min, start, end, sample_num)
        for idx in range(sample_num):
            vertex = vertices[idx]
            if self.planning_env.state_validity_checker(np.array([vertex])):
                self.G, _ = graph_maker.add_node(self.G, vertex, env=self.planning_env,
                connection_radius=min_cost)
                added_nodes.append(vertex)
                start_time = self.tic_toc(time.time() - start_time)
        return added_nodes, ellipse

    def plan_multi_goals(self, start):
        """
        Plan a route from start to self.goals
        Return a path connecting them.
        """
        path = [] # contains nodes
        print("start plan multiple goals")
        for goal in self.goals:
            path.append(self.plan_to_goal(start, goal))
            start = goal
        return np.concatenate(path, axis=0)

    def get_goal(self, msg):
        """
        - plan in map frame  - execute in world frame
        don't forget tranform back to world frame
        """
        print("Got a new goal\n{}".format(msg.pose))
        goal_pose = util.rospose_to_posetup(msg.pose)
        self.goal = self.world2map(np.array([goal_pose]))[0]

        start_pose = util.rospose_to_posetup(self.current_pose)
        self.start = self.world2map(np.array([start_pose]))[0]
        print("goal and start:", self.goal, self.start)

        map_points = None

        if self.multi_goals:
            if self.route_sent:
                self.route_sent = False
                self.goals = []
            self.goals += [self.goal.copy()]
            if len(self.goals) == self.num_goals:
                print("Got the final goal for mutli goal. Planning for multiple goals")
                map_points = self.plan_multi_goals(self.start)
            else:
                print("Got {}/{} goals.".format(len(self.goals), self.num_goals))
        else:
            print("start single goal")
            map_points = self.plan_to_goal(self.start, self.goal)
        if map_points is not None and len(map_points) > 0:
            world_points = self.map2world(map_points)
            success = self.send_path(np.array(world_points))
            print("Sent path")

    def get_current_pose(self, msg):
        self.current_pose = msg.pose
        if self.goals and len(self.goals) == self.num_goals:
            self.goals = [] # clear
        self.goal = None

    def send_path(self, waypoints):
        print("prepare to send waypoints", len(waypoints))
        h = Header()
        h.stamp = rospy.Time.now()
        desired_speed = 1.0

        speeds = np.zeros(len(waypoints))
        speeds[:] = desired_speed
        speeds[-1] = 0.0
        path = XYHVPath(h,[XYHV(*[waypoint[0], waypoint[1], waypoint[2], speed]) \
                for waypoint, speed in zip(waypoints, speeds)])

        print "Sending path..."
        controller = rospy.ServiceProxy("/controller/follow_path", FollowPath())
        success = controller(path)
        print "Controller started.", success
        return success

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

    def world2map(self, poses):
        '''
        world2map is a utility function which converts poses from global
            'world' coordinates (ROS world frame coordinates) to 'map'
            coordinates, that is pixel frame.
        input:
            poses - set of X input poses
            out - output buffer to load converted poses into
        '''
        out = poses.copy()
        out[:] = poses
        # translation
        out[:, 0] -= self.map_x
        out[:, 1] -= self.map_y

        # scale
        out[:, :2] *= (1.0 / float(self.map.info.resolution))

        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(out[:, 0])
        out[:, 0] = self.map_c * out[:, 0] - self.map_s * out[:, 1]
        out[:, 1] = self.map_s * temp + self.map_c * out[:, 1]
        out[:, 2] += self.map_angle
        return out

    '''
    Convert an array of pixel locations in the map to poses in the world. Does computations
    in-place
      poses: Pixel poses in the map. Should be a nx3 numpy array
      map_info: Info about the map (returned by get_map)
    '''
    def map2world(self, poses):
        scale = self.map.info.resolution
        angle = -self.map_angle

        # Rotation
        c, s = np.cos(angle), np.sin(angle)

        # Store the x coordinates since they will be overwritten
        temp = np.copy(poses[:,0])
        poses[:,0] = c*poses[:,0] - s*poses[:,1]
        poses[:,1] = s*temp       + c*poses[:,1]

        # Scale
        poses[:,:2] *= float(scale)

        # Translate
        poses[:,0] += self.map_x
        poses[:,1] += self.map_y
        poses[:,2] += angle

        return poses


    def tight_sampling_limits(self, env):
        """
        reset the x, y limits of a environment to
        ones that tightly bounds the permissible area.

        @param env: the environment whose x, y limits are to be updated
        @modifies env
        """
        xs, ys = np.where(env.map == 0)
        env.xlimit = [xs.min(), xs.max()+1]
        env.ylimit = [ys.min(), ys.max()+1]
        env.limit = np.array([env.xlimit, env.ylimit])
        print env.map.shape
        print xs.min(), xs.max()+1
        print ys.min(), ys.max()+1



if __name__ == '__main__':
    do_shortcut = rospy.get_param("planner/do_shortcut", False)
    num_goals = int(rospy.get_param("planner/goals", 1))
    heuristic = lambda n1, n2, env, G: env.compute_heuristic(n1, n2)
    weight = lambda n1, n2, env, G: env.edge_validity_checker(n1, n2)
    ROSPlanner(heuristic, weight, num_vertices=30, connection_radius=1000, do_shortcut=do_shortcut, num_goals=num_goals, 
            plan_time=2, plan_with_budget=True) # 250 500
