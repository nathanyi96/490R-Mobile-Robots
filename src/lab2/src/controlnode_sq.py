import rospy
import threading

import numpy as np
import os
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float32
from std_srvs.srv import Empty as SrvEmpty
from lab2.msg import XYHVPath, XYHVPath_log
from lab2.srv import FollowPath, FollowPath_log, StampedFollowPath
from visualization_msgs.msg import Marker

import csv
import mpc
import mpc2
import nonlinear
import pid
import purepursuit
import utils

controllers = {
    "PID": pid.PIDController,
    "PP": purepursuit.PurePursuitController,
    "NL": nonlinear.NonLinearController,
    "MPC": mpc.ModelPredictiveController,
    "MPC2": mpc2.ModelPredictiveControllerNaive,
}

class AverageMeter(object):
    """Computes and stores the average and current value"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count

    def __repr__(self):
        return '{:.3f} ({:.3f})'.format(self.val, self.avg)

class ControlNode:
    def __init__(self, name):
        self.ackermann_msg_id = 0

        self.path_event = threading.Event()
        self.reset_lock = threading.Lock()
        self.ready_event = threading.Event()
        self.error_logger = AverageMeter()
        self.initialpose = np.zeros(3)
        self.start(name)

    def log_error(self):
        rospy.loginfo(self.path_name)
        rospy.loginfo(self.initialpose[0])
        kp = float(rospy.get_param("/pid/kp", 3.0)) # 0.65
        kd = float(rospy.get_param("/pid/kd", 2.0)) # 0.8
        L = float(rospy.get_param("/purepursuit/pose_lookahead", 1.0))
        K = int(rospy.get_param("/mpc/K", 140))
        T = int(rospy.get_param("/mpc/T", 5))
        k1 = float(rospy.get_param("/lyapunov/k1", 5.0))
        k2 = float(rospy.get_param("/lyapunov/k2", 10.0))
        # with open(os.path.dirname(os.path.realpath(__file__)) + 'lab2_performance_log.txt', 'a+') as f:
        #     # writer = csv.writer(f, delimiter=' ')
        #     f.write('{}, {}, {}, {}, {}, {}, {}, {}\n'.format(
        #     self.controller_type, self.error_logger.avg, self.path_name, self.initialpose[0], self.initialpose[1],
        #     self.initialpose[2], kp, kd))
        if self.controller_type == 'PID':
            with open(os.path.dirname(os.path.realpath(__file__)) + '/pid_experiments_log.csv', 'a+') as f:
                f.write('{}, {}, {}, {}, {}, {}, {}, {}\n'.format(
                self.controller_type, self.error_logger.avg, self.path_name, self.initialpose[0], self.initialpose[1],
                self.initialpose[2], kp, kd))
        elif self.controller_type == 'PP':
            with open(os.path.dirname(os.path.realpath(__file__)) + '/pp_r_v_experiments_log.csv', 'a+') as f:
                f.write('{}, {}, {}, {}, {}, {}, {} \n'.format(
                self.controller_type, self.error_logger.avg, self.path_name, self.initialpose[0], self.initialpose[1],
                self.initialpose[2], L))
        elif self.controller_type == 'MPC':
            with open(os.path.dirname(os.path.realpath(__file__)) + '/mpc_experiments_log.csv', 'a+') as f:
                f.write('{}, {}, {}, {}, {}, {}, {}, {}\n'.format(
                self.controller_type, self.error_logger.avg, self.path_name, self.initialpose[0], self.initialpose[1],
                self.initialpose[2], K, T))
        elif self.controller_type == 'NL':
            with open(os.path.dirname(os.path.realpath(__file__)) + '/NL_experiments_log.csv', 'a+') as f:
                f.write('{}, {}, {}, {}, {}, {}, {}, {}\n'.format(
                self.controller_type, self.error_logger.avg, self.path_name, self.initialpose[0], self.initialpose[1],
                self.initialpose[2], k1, k2))
        self.error_logger.reset()

    def start(self, name):
        rospy.init_node(name, anonymous=True, disable_signals=True)

        self.setup_pub_sub()
        self.load_controller()
        self.ready_event.set()

        rate = rospy.Rate(50) # 50 in sim, 8 in real with PF
        self.inferred_pose = None
        print "Control Node Initialized"
        rospy.loginfo('start')
        while not rospy.is_shutdown():
            self.path_event.wait()
            self.reset_lock.acquire()
            rospy.loginfo('run')
            ip = self.inferred_pose
            if ip is not None and self.controller.ready():
                index = self.controller.get_reference_index(ip)
                pose = self.controller.get_reference_pose(index)
                error = self.controller.get_error(ip, index)
                cte = error[1]
                self.error_logger.update(np.linalg.norm(error))
                self.publish_selected_pose(pose)
                self.publish_cte(cte)
                next_ctrl = self.controller.get_control(ip, index)
                if next_ctrl is not None:
                    self.publish_ctrl(next_ctrl)
                if self.controller.path_complete(ip, error):
                    # self.log_error()
                    self.path_event.clear()
                    self.notifyComplete()
                    # call manager/complete service
            self.reset_lock.release()
            rate.sleep()

    def shutdown(self):
        rospy.signal_shutdown("shutting down from signal")
        self.path_event.clear()
        self.ready_event.clear()
        exit(0)

    def load_controller(self):
        self.controller_type = rospy.get_param("/controller/type", default="PP")
        self.controller = controllers[self.controller_type]()

    def setup_pub_sub(self):
        rospy.Service("~reset/hard", SrvEmpty, self.srv_reset_hard)
        rospy.Service("~reset/state", SrvEmpty,  self.srv_reset_state)
        rospy.Service("~reset/params", SrvEmpty, self.srv_reset_params)

        rospy.Subscriber("/initialpose",
                PoseWithCovarianceStamped, self.cb_init_pose, queue_size=1)
        rospy.Subscriber("/controller/set_path",
                 XYHVPath, self.cb_path, queue_size=1)
        # rospy.Subscriber("/controller/set_path",
        #         XYHVPath_log, self.cb_path_log, queue_size=1)
        rospy.Service("/controller/follow_path", FollowPath, self.cb_path)
        #rospy.Service("/controller/follow_path_log", FollowPath_log, self.cb_path_log)

        self.notifyComplete = rospy.ServiceProxy("path_manager/path_complete", SrvEmpty)

        rospy.Subscriber(rospy.get_param("~pose_topic", "/sim_car_pose/pose"),
                             PoseStamped, self.cb_pose, queue_size=10)

        self.rp_ctrls = rospy.Publisher(
            rospy.get_param(
                "~ctrl_topic",
                default="/vesc/high_level/ackermann_cmd_mux/input/nav_0"
            ),
            AckermannDriveStamped, queue_size=2
        )

        self.rp_cte = rospy.Publisher(
            rospy.get_param(
                "~cte_viz_topic",
                default="/controller/cte"
            ),
            Float32, queue_size=2
        )

        self.rp_waypoints = rospy.Publisher(
            rospy.get_param(
                "~waypoint_viz_topic",
                default="/controller/path/waypoints"
            ),
            Marker, queue_size=2
        )

        self.rp_waypoint = rospy.Publisher(
            rospy.get_param(
                "~selected_waypoint_viz_topic",
                default="/controller/path/selected_waypoint"
            ),
            PoseStamped, queue_size=2
        )

        self.rp_path_viz = rospy.Publisher(
            rospy.get_param(
                "~poses_viz_topic",
                default="/controller/path/poses"
            ),
            PoseArray, queue_size=2
        )

    def srv_reset_hard(self, msg):
        '''
        Hard reset does a complete reload of the controller.
        '''
        rospy.loginfo("Start hard reset")
        self.reset_lock.acquire()
        self.load_controller()
        self.reset_lock.release()
        rospy.loginfo("End hard reset")
        return []

    def srv_reset_params(self, msg):
        '''
        Param reset resets parameters of the controller. Useful for iterative tuning.
        '''
        rospy.loginfo("Start param reset")
        self.reset_lock.acquire()
        self.controller.reset_params()
        self.reset_lock.release()
        rospy.loginfo("End param reset")
        return []

    def srv_reset_state(self, msg):
        '''
        State reset resets state dependent variables, such as accumulators in PID control.
        '''
        rospy.loginfo("Start state reset")
        self.reset_lock.acquire()
        self.controller.reset_state()
        self.reset_lock.release()
        rospy.loginfo("End state reset")
        return []

    def cb_odom(self, msg):
        self.inferred_pose = utils.rospose_to_posetup(msg.pose.pose)

    def cb_path(self, msg):

        rospy.loginfo("Got path!")
        path = msg.path.waypoints
        print("path", path)
        self.visualize_path(path)
        self.controller.set_path(path)
        self.path_event.set()
        print "Path set"
        return True

    def cb_path_log(self, msg):
        rospy.loginfo("Got path!!")
        path = msg.path.waypoints
        rospy.loginfo('path'+str(path))
        self.path_name = msg.path_name
        rospy.loginfo(self.path_name)
        self.visualize_path(path)
        self.controller.set_path(path)
        self.path_event.set()
        rospy.loginfo("Path set")
        return True

    def cb_pose(self, msg):
        self.inferred_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            utils.rosquaternion_to_angle(msg.pose.orientation)]

    def publish_ctrl(self, ctrl):
        assert len(ctrl) == 2
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.header.seq = self.ackermann_msg_id
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        # rospy.loginfo("v: {}, delta: {}".format(ctrl[0], ctrl[1]))
        self.rp_ctrls.publish(ctrlmsg)
        self.ackermann_msg_id += 1

    def visualize_path(self, path):
        marker = self.make_marker(path[0], 0, "start")
        self.rp_waypoints.publish(marker)
        poses = []
        for i in range(1, len(path)):
            p = Pose()
            p.position.x = path[i].x
            p.position.y = path[i].y
            p.orientation = utils.angle_to_rosquaternion(path[i].h)
            poses.append(p)
        pa = PoseArray()
        pa.header = Header()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"
        pa.poses = poses
        self.rp_path_viz.publish(pa)

    def make_marker(self, config, i, point_type):
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = str(config)
        marker.id = i
        marker.type = Marker.CUBE
        marker.pose.position.x = config.x
        marker.pose.position.y = config.y
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        if point_type == "waypoint":
            marker.color.b = 1.0
        else:
            marker.color.g = 1.0

        return marker

    def publish_selected_pose(self, pose):
        p = PoseStamped()
        p.header = Header()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.orientation = utils.angle_to_rosquaternion(pose[2])
        self.rp_waypoint.publish(p)

    def publish_cte(self, cte):
        self.rp_cte.publish(Float32(cte))

    def cb_init_pose(self, pose):
        # self.initialpose = utils.rospose_to_posetup_(pose)
        self.path_event.clear()
        self.path_event.set()
