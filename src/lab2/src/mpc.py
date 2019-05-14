import numpy as np
import rospy
import utils

from controller import BaseController

from nav_msgs.srv import GetMap

import time
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from scipy.spatial.distance import directed_hausdorff
from scipy.spatial.distance import cdist
from scipy.stats import truncnorm
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, PointStamped
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import os

SCAN_TOPIC = "/scan" # Laser scan topic for actual laser scan

class ModelPredictiveController(BaseController):
    def __init__(self):
        super(ModelPredictiveController, self).__init__()
        self.traj_pub = rospy.Publisher("/mpc/traj", Path, queue_size=10)

        self.reset_params()
        self.reset_state()
        self.init_laser()

        rospy.loginfo('plotting traj lib')
        self.visualize_traj()

    def get_reference_index(self, pose):
        '''
        get_reference_index finds the index i in the controller's path
            to compute the next control control against
        input:
            pose - current pose of the car, represented as [x, y, heading]
        output:
            i - referencence index
        '''
        with self.path_lock:
            # TODO: INSERT CODE HERE
            # Determine a strategy for selecting a reference point
            # in the path. One option is to find the nearest reference
            # point and chose the next point some distance away along
            # the path. You are welcome to use the same method for MPC
            # as you used for PID or Pure Pursuit.
            
            # dist = np.sum((self.path[:,0:2] - pose[0:2])**2, axis=1)
            # dist = np.sqrt(dist)
            # argm = np.argmin(dist)
            # #print "min dist = ref[", argm, "] =", dist[argm]
            # closest = np.abs(dist[argm:] - self.waypoint_lookahead)
            # idx = np.argmin(closest) + argm
            # return idx
            dist = np.sqrt(np.sum(((self.path[:,0:2] - pose[0:2])**2), axis=1))
            idx = np.argmin(dist)
            return (idx if (idx == len(self.path)-1) else idx+1)


    def get_control(self, pose, index):
        '''
        get_control - computes the control action given an index into the
            reference trajectory, and the current pose of the car.
            Note: the output velocity is given in the reference point.
        input:
            pose - the vehicle's current pose [x, y, heading]
            index - an integer corresponding to the reference index into the
                reference path to control against
        output:
            control - [velocity, steering angle]
        '''
        assert len(pose) == 3

        # TODO: INSERT CODE HERE
        # With MPC, you should roll out K control trajectories using
        # the kinematic car model, then score each one with your cost
        # function, and finally select the one with the lowest cost.

        pose_ref = self.get_reference_pose(index)
        if self.ENABLE_DYNAMIC_TRAJ:
            prev_T = self.T
            if np.linalg.norm(pose[:2]-pose_ref[:2]) > self.TRAJLIB_THRES:
                self.T = self.Tshort
                self.trajs = self.trajs_short
                self.scaled = self.scaled_short
                self.bbox_map = self.bbox_map_short
                self.perm = self.perm_short
            else:
                self.T = self.Tlong
                self.trajs = self.trajs_long
                self.scaled = self.scaled_long
                self.bbox_map = self.bbox_map_long
                self.perm = self.perm_long
                
        rollouts = np.zeros((self.K, 1+self.T*self.pose_resol, 3))

        # For each K trial, the first position is at the current position (pose)
        # Set initial 
        rollouts[:,0,0] = pose[0] # x
        rollouts[:,0,1] = pose[1] # y
        rollouts[:,0,2] = pose[2] # theta

        # For control trajectories, get the speed from reference point.
        self.trajs[:,:,0] = pose_ref[3] # Update velocities

        # Then step forward the K control trajectory T timesteps using
        # self.apply_kinematics
        dt = self.dt / self.pose_resol
        for i in xrange(self.T):
            for j in xrange(self.pose_resol):
                it = self.pose_resol*i + j
                rollouts[:,it+1,:] = np.array(self.apply_kinematics(rollouts[:,it,:], self.trajs[:,i,:], dt)).T # ...(positions, controls)

        # Apply the cost function to the rolled out poses.
        costs = self.apply_cost(rollouts, pose, index)

        # Finally, find the control trajectory with the minimum cost.
        min_control = np.argmin(costs)
        # print 'costs', costs
        # print 'min control', min_control

        # Return the controls which yielded the min cost.
        #rospy.loginfo(str(rollouts[min_control]))
        self.visualize_selected_traj(rollouts[min_control])
        return self.trajs[min_control][0]

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        with self.path_lock:
            self.trajs = self.get_control_trajectories(self.T)
            assert self.trajs.shape == (self.K, self.T, 2)
            self.scaled = np.zeros((self.K * self.T, 3))
            self.bbox_map = np.zeros((self.K * self.T, 2, 4))
            self.perm = np.zeros(self.K * self.T).astype(np.int)
            self.map = self.get_map()
            self.perm_reg = self.load_permissible_region(self.map)
            self.map_x = self.map.info.origin.position.x
            self.map_y = self.map.info.origin.position.y
            self.map_angle = utils.rosquaternion_to_angle(self.map.info.origin.orientation)
            self.map_c = np.cos(self.map_angle)
            self.map_s = np.sin(self.map_angle)
            if self.ENABLE_DYNAMIC_TRAJ:
                self.trajs_long = self.trajs
                self.trajs_short = self.get_control_trajectories(self.Tshort)
                self.scaled_long = self.scaled
                self.bbox_map_long = self.bbox_map
                self.perm_long = self.perm
                self.scaled_short = self.scaled[:self.K*self.Tshort,:]
                self.bbox_map_short = self.bbox_map[:self.K*self.Tshort,:,:]
                self.perm_short = self.perm[:self.K*self.Tshort]

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.wheelbase = float(rospy.get_param("trajgen/wheelbase", 0.33))
            self.min_delta = float(rospy.get_param("trajgen/min_delta", -0.34))
            self.max_delta = float(rospy.get_param("trajgen/max_delta", 0.34))

            self.K = int(rospy.get_param("mpc/K", 140))
            self.T = int(rospy.get_param("mpc/T", 6))

            self.speed = float(rospy.get_param("mpc/speed", 1.0))
            self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 1.0))
            self.exceed_threshold = float(rospy.get_param("mpc/exceed_threshold", 100.0))
            # Average distance from the current reference pose to lookahed.
            self.waypoint_lookahead = float(rospy.get_param("mpc/waypoint_lookahead", 1.5))
            self.collision_w = float(rospy.get_param("mpc/collision_w", 1e5))
            self.error_w = float(rospy.get_param("mpc/error_w", 1.0))

            self.car_length = float(rospy.get_param("mpc/car_length", 0.33))
            self.car_width = float(rospy.get_param("mpc/car_width", 0.15))

            # custom parameters
            self.dt = float(rospy.get_param("mpc/dt", 0.2))
            self.shape_w = float(rospy.get_param("mpc/shape_w", 8.0))
            self.num_branches = int(rospy.get_param("mpc/num_branches", 7))
            self.pose_resol = int(rospy.get_param("mpc/pose_resolution", 2))
            self.ENABLE_DYNAMIC_TRAJ = rospy.get_param("mpc/dynamic_traj", True)
            if self.ENABLE_DYNAMIC_TRAJ:
                self.Tlong = self.T
                self.Tshort = 3
                self.TRAJLIB_THRES = 1.0

    def init_laser(self):
        self.EXCLUDE_MAX_RANGE_RAYS = True
        self.OBS_STEPS = 3
        self.LASER_RAY_STEP = 18
        self.MAX_RANGE_METERS = 11.0
        self.OBS_SAFE_DIST = 0.3
        assert self.OBS_STEPS <= self.T

        self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.lidar_cb, queue_size=1) # Callback at end of this file
        self.obs_pub = rospy.Publisher("/mpc/obs_pos", Marker, queue_size=30)

        self.laser_angles = None # The angles of each ray
        self.downsampled_angles = None # The angles of the downsampled rays
        self.downsampled_ranges = None # The ranges of the downsampled rays
        self.filtered_angles = None
        self.filtered_ranges = None
        self.obs = None
        self.obs_w = 3.0

    def get_control_trajectories2(self):
        '''
        get_control_trajectories computes K control trajectories to be
            rolled out on each update step. You should only execute this
            function when initializing the state of the controller.

            various methods can be used for generating a sufficient set
            of control trajectories, but we suggest stepping over the
            control space (of steering angles) to create the initial
            set of control trajectories.
        output:
            ctrls - a (K x T x 2) vector which lists K control trajectories
                of length T
        '''
        # TODO: INSERT CODE HERE
        # Create a trajectory library of K trajectories for T timesteps to
        # roll out when computing the best control.
        ctrls = np.zeros((self.K, self.T, 2))
        # step_size = (self.max_delta - self.min_delta) / (self.K - 1)

        sd = 0.8
        mean = 0
        lb, ub = (self.min_delta - mean) / sd, (self.max_delta - mean) / sd
        tnormal = truncnorm(lb, ub, loc=mean, scale=sd)
        for i in xrange(self.T):
            ctrls[:,i,1] = tnormal.rvs(size=self.K)  # sample from control space with truncated gaussian
        ctrls[0,:,1] = np.zeros(self.T)
        return ctrls

    def get_control_trajectories(self, T):
        '''
        get_control_trajectories computes K control trajectories to be
            rolled out on each update step. You should only execute this
            function when initializing the state of the controller.

            various methods can be used for generating a sufficient set
            of control trajectories, but we suggest stepping over the
            control space (of steering angles) to create the initial
            set of control trajectories.
        output:
            ctrls - a (K x T x 2) vector which lists K control trajectories
                of length T
        '''
        # TODO: INSERT CODE HERE
        # Create a trajectory library of K trajectories for T timesteps to
        # roll out when computing the best control.
        assert self.K in range(60, 150, 10)
        if not os.path.exists(os.path.dirname(os.path.realpath(__file__)) + '/../traj_lib/depth_{}_branch_{}_size_{}.npy'.format(T, self.num_branches, self.K)):
            self.precompute_traj_lib(T, self.num_branches)
        return np.load(os.path.dirname(os.path.realpath(__file__)) + '/../traj_lib/depth_{}_branch_{}_size_{}.npy'.format(T, self.num_branches, self.K))
        

    def precompute_traj_lib(self, t, num_branches):
        # generate dense set of paths D
        time_start = time.time()
        deltas = np.linspace(self.min_delta, self.max_delta, num=num_branches, endpoint=True)
        D_size = num_branches**t

        grid = np.meshgrid(*([deltas] * t), indexing='ij')
        D = np.stack([x.flatten() for x in grid], axis=-1)
        D = np.concatenate((D, np.zeros([1, int(t)]))) # straight
        rospy.loginfo('dense generation time:' + str(time.time()-time_start))
        dcache = {}  # dcache[(p1, p2)] = d(p1, p2), where p1 \in D, p2 \in S

        K = range(60, 150, 10)
        time_start = time.time()
        # contruct subset S with (hopefully) maximum coverage
        S = set((D_size,))  # add default to S
        for i in xrange(1, max(K)):
            # find path that (might) makes maximum dispersion after adding it to S
            max_mindist = -1000000
            new_path = None
            for path in xrange(D_size):
                if path not in S:
                    min_dist = 1000000000
                    for path_S in S:
                        # find hausdorff distance in control space. TODO: maybe convert to state space
                        if (path, path_S) in dcache:
                            dist = dcache[(path, path_S)]
                        else:
                            p1 = D[[path]]
                            p2 = D[[path_S]]
                            dist = max(directed_hausdorff(p1, p2)[0], directed_hausdorff(p2, p1)[0])
                            dcache[(path, path_S)] = dist
                        min_dist = min(min_dist, dist)
                    assert min_dist < 1000000000
                    if min_dist > max_mindist:
                        max_mindist = min_dist
                        new_path = path
            assert new_path != None
            S.add(new_path)
            if len(S) in K:
                ctrls = np.zeros((len(S), t, 2))
                cnt = 0
                for path in S:
                    ctrls[cnt,:,1] = D[path,:]
                    cnt += 1
                rospy.loginfo('traj selection time:' + str(time.time()-time_start))
                np.save(os.path.dirname(os.path.realpath(__file__)) + '/../traj_lib/depth_{}_branch_{}_size_{}.npy'.format(t, num_branches, len(S)), ctrls)

    def apply_kinematics(self, cur_x, control, dt):
        '''
        apply_kinematics 'steps' forward the pose of the car using
            the kinematic car model for a given set of K controls.
        input:
            cur_x   (K x 3) - current K "poses" of the car
            control (K x 2) - current controls to step forward
        output:
            (x_dot, y_dot, theta_dot) - where each *_dot is a list
                of k deltas computed by the kinematic car model.
        '''
        # TODO: INSERT CODE HERE
        # Use the kinematic car model discussed in class to step
        # forward the pose of the car. We will step all K poses
        # simultaneously, so we recommend using Numpy to compute
        # this operation.
        #
        # ulimately, return a triplet with x_dot, y_dot_, theta_dot
        # where each is a length K numpy vector.

        x = cur_x[:,0]
        y = cur_x[:,1]
        theta = cur_x[:,2]
        velocity = control[:,0]
        delta = control[:,1]

        theta_dot = theta + (velocity / self.B * np.tan(delta)) * dt
        x_dot = x + (self.B / (np.tan(delta)+1e-12) * (np.sin(theta_dot) - np.sin(theta))) 
        y_dot = y + (self.B / (np.tan(delta)+1e-12) * (-np.cos(theta_dot) + np.cos(theta)))
        straight_idx = (np.absolute(delta) < 1e-8)
        x_dot[straight_idx] = x[straight_idx] + ((np.cos(theta))[straight_idx])*velocity[straight_idx]*dt
        y_dot[straight_idx] = y[straight_idx] + ((np.sin(theta))[straight_idx])*velocity[straight_idx]*dt


        
        return (x_dot, y_dot, theta_dot)

    def apply_cost(self, poses, curr_pose, index):
        '''
        rollouts (K,T*pose_resol+1,3) - poses of each rollout
        index    (int)   - reference index in path
        '''
        path_steps = poses.shape[1]
        all_poses = poses[:,self.pose_resol:path_steps:self.pose_resol].copy()
        assert all_poses.shape == (self.K, self.T, 3)
        all_poses.resize(self.K * self.T, 3)
        collisions = self.check_collisions_in_map(all_poses)
        collisions.resize(self.K, self.T)
        collision_cost = collisions.sum(axis=1) * self.collision_w
        # compute shape costs
        pose_ref = self.get_reference_pose(index)
        path_dist = self.dt * pose_ref[3] * self.T
        end_idx, d = index + 1, 0.0
        while end_idx < self.path.shape[0] and d < path_dist:
            d += np.linalg.norm(self.path[end_idx,:2]-self.path[end_idx-1,:2])
            end_idx += 1
        ref_path = self.path[index:end_idx,:2]
        if d < path_dist:
            rem_d = path_dist - d
            step_dist = (d / (end_idx-index)) if end_idx-1 > index else 0.05
            rem_poses = int(rem_d / step_dist)
            x_end = self.path[end_idx-1,0] + rem_d*np.cos(self.path[end_idx-1,2])
            y_end = self.path[end_idx-1,1] + rem_d*np.sin(self.path[end_idx-1,2])
            append_poses = np.empty((rem_poses, 2))
            append_poses[:,0] = np.linspace(self.path[end_idx-1,0], x_end, num=rem_poses+1, endpoint=True)[1:]
            append_poses[:,1] = np.linspace(self.path[end_idx-1,1], y_end, num=rem_poses+1, endpoint=True)[1:]
            ref_path = np.concatenate((ref_path, append_poses), axis=0)
        shape_cost = np.empty(self.K)
        for i in xrange(self.K):
            prop_path = poses[i,:,:2]
            shape_cost[i] = max(directed_hausdorff(ref_path, prop_path)[0], directed_hausdorff(prop_path, ref_path)[0])
        shape_cost *= self.shape_w
        error_cost = np.linalg.norm(poses[:, path_steps - 1, :2] - self.path[index, :2], axis=1) * self.error_w
        obs_cost = self.obstacle_cost(curr_pose, poses[:,:self.OBS_STEPS*self.pose_resol+1,:]) * self.obs_w
        rospy.loginfo('shape cost' + str(shape_cost))
        rospy.loginfo('error cost' + str(error_cost))
        rospy.loginfo('obs cost' + str(obs_cost))
        return collision_cost + error_cost + shape_cost + obs_cost

    def check_collisions_in_map(self, poses):
        '''
        check_collisions_in_map is a collision checker that determines whether a set of K * T poses
            are in collision with occupied pixels on the map.
        input:
            poses (K * T x 3) - poses to check for collisions on
        output:
            collisions - a (K * T x 1) float vector where 1.0 signifies collision and 0.0 signifies
                no collision for the input pose with corresponding index.
        '''

        self.world2map(poses, out=self.scaled)

        L = self.car_length
        W = self.car_width

        # Specify specs of bounding box
        bbox = np.array([
            [L / 2.0, W / 2.0],
            [L / 2.0, -W / 2.0],
            [-L / 2.0, W / 2.0],
            [-L / 2.0, -W / 2.0]
        ]) / (self.map.info.resolution)

        x = np.tile(bbox[:, 0], (len(poses), 1))
        y = np.tile(bbox[:, 1], (len(poses), 1))

        xs = self.scaled[:, 0]
        ys = self.scaled[:, 1]
        thetas = self.scaled[:, 2]

        c = np.resize(np.cos(thetas), (len(thetas), 1))
        s = np.resize(np.sin(thetas), (len(thetas), 1))

        self.bbox_map[:, 0] = (x * c - y * s) + np.tile(np.resize(xs, (len(xs), 1)), 4)
        self.bbox_map[:, 1] = (x * s + y * c) + np.tile(np.resize(ys, (len(ys), 1)), 4)

        bbox_idx = self.bbox_map.astype(np.int)

        self.perm[:] = 0
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 0], bbox_idx[:, 0, 0]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 1], bbox_idx[:, 0, 1]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 2], bbox_idx[:, 0, 2]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 3], bbox_idx[:, 0, 3]])

        return self.perm.astype(np.float)

    def get_map(self):
        '''
        get_map is a utility function which fetches a map from the map_server
        output:
            map_msg - a GetMap message returned by the mapserver
        '''
        srv_name = rospy.get_param("static_map", default="/static_map")
        rospy.logdebug("Waiting for map service")
        rospy.wait_for_service(srv_name)
        rospy.logdebug("Map service started")

        map_msg = rospy.ServiceProxy(srv_name, GetMap)().map
        return map_msg

    def load_permissible_region(self, map):
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
        pr = np.zeros_like(array_255, dtype=bool)

        # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        pr[array_255 == 0] = 1
        pr = np.logical_not(pr)  # 0 is permissible, 1 is not

        return pr

    def world2map(self, poses, out):
        '''
        world2map is a utility function which converts poses from global
            'world' coordinates (ROS world frame coordinates) to 'map'
            coordinates, that is pixel frame.
        input:
            poses - set of X input poses
            out - output buffer to load converted poses into
        '''
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

    def visualize_selected_traj(self, poses):
        #rospy.loginfo("poses length" + str(poses.shape))
        path_poses = []
        assert self.T*self.pose_resol+1 == poses.shape[0]
        time_now = rospy.Time.now()
        for i in xrange(poses.shape[0]):
            pose = PoseStamped()
            #pose.header = Header()
            #pose.header.stamp = time_now
            #pose.header.frame_id = "map"
            pose.pose.position.x = poses[i,0]
            pose.pose.position.y = poses[i,1]
            pose.pose.position.z = 0
            pose.pose.orientation = utils.angle_to_rosquaternion(poses[i,2])
            path_poses.append(pose)
        path = Path()
        path.header = Header()
        path.header.stamp = time_now
        path.header.frame_id = "map"
        path.poses = path_poses
        self.traj_pub.publish(path)

    def visualize_traj(self):  # must be called after get_control_trajectories()
        fig, ax = plt.subplots()
        rollouts = np.zeros((self.K, self.T+1, 3))
        rollouts[:,0,0] = 0
        rollouts[:,0,1] = 0
        rollouts[:,0,2] = np.pi/2
        self.trajs[:,:,0] = 2.0
        for i in xrange(self.T):              
            rollouts[:,i+1,:] = np.array(self.apply_kinematics(rollouts[:,i,:], self.trajs[:,i,:], self.dt)).T
        for i in xrange(self.K):
            self.plot_path(ax, rollouts[i])
        plt.show()

    def plot_path(self, ax, poses):
        '''
        poses: (T x 3) ndarray, where each pose is (x, y, theta), theta in [-pi, pi]
        '''
        Path = mpath.Path

        for i in xrange(poses.shape[0]):
          ax.arrow(poses[i,0], poses[i,1], 0.05*np.cos(poses[i,2]), 0.05*np.sin(poses[i,2]), width=0.003)

        path_verts = [poses[0,:2]]
        path_codes = [Path.MOVETO]
        for i in xrange(1, poses.shape[0]):
          heading_diff = abs(poses[i-1,2] - poses[i,2])
          if heading_diff < 1e-8 or abs(heading_diff-2*np.pi) < 1e-8:
            path_verts.append(poses[i,:2])
            path_codes.append(Path.LINETO)
          else:
            ct1, ct2 = self.get_control_points(poses[i-1], poses[i])
            path_verts.extend((ct1, ct2, poses[i,:2]))
            path_codes.extend((Path.CURVE4,)*3)
        #return path_verts, path_codes
        pp1 = mpatches.PathPatch(
          Path(path_verts,
               path_codes),
          fc="none", transform=ax.transData)
        ax.add_patch(pp1)
        ax.plot([0], [0], "ro")
        ax.set_title('The red point is the initial location')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-0.1, 3)
        ax.set_aspect(aspect='equal', adjustable='box')

    def get_control_points(self, pose1, pose2):  # helper method for visualization
        phi = pose1[2]-pose2[2]
        if phi > np.pi:
            phi = phi - 2*np.pi
        elif phi < -np.pi:
            phi = phi + 2*np.pi
        L = np.sqrt((pose2[0]-pose1[0])**2 + (pose2[1]-pose1[1])**2)
        R = L / (2 * np.sin(abs(phi/2)))
        ofs_angle_sign = np.pi/2 if phi > 0 else -np.pi/2
        ofs_angle = (pose2[2] + phi/2) + ofs_angle_sign
        while ofs_angle > np.pi: ofs_angle -= 2*np.pi
        while ofs_angle < -np.pi: ofs_angle += 2*np.pi
        dir1 = pose1[2]-ofs_angle_sign
        dir2 = pose2[2]-ofs_angle_sign
        ofs_pos = np.array((R*np.cos(dir1), R*np.sin(dir1))) + pose1[:2]
        ofs_pos2 = np.array((R*np.cos(dir2), R*np.sin(dir2))) + pose2[:2]
        #print(R)
        #print(ofs_angle, np.pi/4)
        #print(dir1, dir2)
        #print(ofs_pos, ofs_pos2)
        assert np.linalg.norm(ofs_pos - ofs_pos2) < 1e-8
        # get control points of simple arc
        P0 = np.array( (np.cos(phi/2), np.sin(phi/2)) )
        #P3 = np.array( (P0[0], -P0[1]) )
        P1 = np.array( ((4-P0[0]) / 3, (1-P0[0]) * (3-P0[0])/(3 * P0[1])) )
        P2 = np.array( (P1[0], -P1[1]) )
        # apply transformation to control points
        c, s = np.cos(ofs_angle), np.sin(ofs_angle)
        rot_mat = np.array([[c, -s], [s, c]])
        P0 = np.matmul(rot_mat, R*P0) + ofs_pos
        P1 = np.matmul(rot_mat, R*P1) + ofs_pos
        P2 = np.matmul(rot_mat, R*P2) + ofs_pos
        #P3 = np.matmul(rot_mat, R*P3) + ofs_pos
        #print(P0, pose1)
        #print(P3, pose2)
        assert np.linalg.norm(P0 - pose1[:2]) < 1e-8
        #assert np.linalg.norm(P3 - pose2[:2]) < 1e-8
        return P1, P2

    # Copied from lab1 SensorModel.py
    def lidar_cb(self, msg):
        # Down sample the laser rays
        if not self.EXCLUDE_MAX_RANGE_RAYS:
          # Initialize angle arrays
            if not isinstance(self.laser_angles, np.ndarray):
                self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
                self.downsampled_angles = np.copy(self.laser_angles[0::self.LASER_RAY_STEP]).astype(np.float32)

            self.downsampled_ranges = np.array(msg.ranges[::self.LASER_RAY_STEP]) # Down sample
            self.downsampled_ranges[np.isnan(self.downsampled_ranges)] = self.MAX_RANGE_METERS # Remove nans
            self.downsampled_ranges[self.downsampled_ranges[:] == 0] = self.MAX_RANGE_METERS # Remove 0 values
        else:    
            # Initialize angle array
            if not isinstance(self.laser_angles, np.ndarray):
                self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))    
            ranges = np.array(msg.ranges) # Get the measurements
            ranges[np.isnan(ranges)] = self.MAX_RANGE_METERS # Remove nans
            valid_indices = np.logical_and(ranges > 0.01, ranges < self.MAX_RANGE_METERS) # Find non-extreme measurements
            self.filtered_angles = np.copy(self.laser_angles[valid_indices]).astype(np.float32) # Get angles corresponding to non-extreme measurements
            self.filtered_ranges = np.copy(ranges[valid_indices]).astype(np.float32) # Get non-extreme measurements
        
            ray_count = int(self.laser_angles.shape[0]/self.LASER_RAY_STEP) # Compute expected number of rays
            sample_indices = np.arange(0,self.filtered_angles.shape[0],float(self.filtered_angles.shape[0])/ray_count).astype(np.int) # Get downsample indices
            self.downsampled_angles = np.zeros(ray_count+1, dtype=np.float32) # Initialize down sample angles
            self.downsampled_ranges = np.zeros(ray_count+1, dtype=np.float32) # Initialize down sample measurements
            self.downsampled_angles[:sample_indices.shape[0]] = np.copy(self.filtered_angles[sample_indices]) # Populate downsample angles
            self.downsampled_ranges[:sample_indices.shape[0]] = np.copy(self.filtered_ranges[sample_indices]) # Populate downsample measurements
       
        # Compute the observation
        # obs is a a two element tuple
        # obs[0] is the downsampled ranges
        # obs[1] is the downsampled angles
        # Each element of obs must be a numpy array of type np.float32
        # Use self.LASER_RAY_STEP as the downsampling step
        # Keep efficiency in mind, including by caching certain things that won't change across future iterations of this callback
        self.obs = (np.copy(self.downsampled_ranges).astype(np.float32), self.downsampled_angles.astype(np.float32))

    def obstacle_cost(self, curr_pose, poses):
        rospy.loginfo('num beams %s %s', self.obs[0].shape[0], self.obs[1].shape[0])
        filtered_thres = self.OBS_STEPS * 2.0 * self.dt
        assert filtered_thres < self.MAX_RANGE_METERS
        obs_indices = (self.obs[0] <= filtered_thres)
        obs = (self.obs[0][obs_indices], self.obs[1][obs_indices])
        obs_h = obs[1] + curr_pose[2]
        obs_pos = np.stack((curr_pose[0] + obs[0]*np.cos(obs_h), curr_pose[1] + obs[0]*np.sin(obs_h)), axis=1)
        self.plot_obs(obs_pos)
        positions = poses[:,:,:2].copy()
        positions.resize(poses.shape[0] * poses.shape[1], 2)
        assert positions.shape == (poses.shape[0] * poses.shape[1], 2)
        pose_obs_dist = cdist(positions, obs_pos, 'euclidean')
        pose_obs_dist.resize(poses.shape[0], poses.shape[1], obs_pos.shape[0])
        assert pose_obs_dist.shape == (self.K, poses.shape[1], obs_pos.shape[0])
        pose_costs = np.zeros((self.K, poses.shape[1], obs_pos.shape[0]))
        danger_indices = pose_obs_dist < self.OBS_SAFE_DIST
        pose_costs[danger_indices] = self.OBS_SAFE_DIST - pose_obs_dist[danger_indices]
        final_costs = np.sum(np.sum(pose_costs, axis=2), axis=1)
        return final_costs # Array of size K

    def plot_obs(self, obs_pos):
        time_now = rospy.Time.now()
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = time_now
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose = Pose()
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        points = []
        for i in xrange(obs_pos.shape[0]):
            p = Point()
            p.x = obs_pos[i,0]
            p.y = obs_pos[i,1]
            points.append(p)
        marker.points = points
        self.obs_pub.publish(marker)
