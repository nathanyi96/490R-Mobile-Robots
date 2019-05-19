#!/usr/bin/env python
from lab2.msg import XYHV, XYHVPath, XYHVPath_log
from lab2.srv import FollowPath, FollowPath_log
from std_msgs.msg import Header

import numpy as np
import pickle
from scipy import signal
import rospy
import os

def saw():
    t = np.linspace(0, 20, 100)
    saw = signal.sawtooth(0.5 * np.pi * t)
    configs = [[x, y, 0] for (x, y) in zip(t, saw)]
    return configs


def circle():
    # Offset 
    offset = np.array([[-2.1, -0.4, 0]])
    waypoint_sep = 0.1
    radius = 2.5
    center = [0, radius]
    num_points = int((2 * radius * np.pi) / waypoint_sep)
    thetas = np.linspace(-1 * np.pi / 2, 2 * np.pi - (np.pi / 2), num_points)
    poses = [[radius * np.cos(theta) + center[0], radius * np.sin(theta) + center[1], theta + (np.pi / 2)] for theta in thetas]
    return np.array(poses) + offset 


def left_turn():
    offset = np.array([[-7.0, -1.7, 0]])
    waypoint_sep = 0.1
    turn_radius = 1.5
    straight_len = 5.0
    turn_center = [straight_len, turn_radius]
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]
    num_turn_points = int((turn_radius * np.pi * 0.5) / waypoint_sep)
    thetas = np.linspace(-1 * np.pi / 2, 0, num_turn_points)
    turn_poses = [[turn_radius * np.cos(theta) + turn_center[0], turn_radius * np.sin(theta) + turn_center[1], theta + (np.pi / 2)] for theta in thetas]
    poses = straight_poses + turn_poses
    return np.array(poses) + offset


def right_turn():
    waypoint_sep = 0.1
    turn_radius = 1.5
    straight_len = 10.0
    turn_center = [straight_len, -turn_radius]
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]
    num_turn_points = int((turn_radius * np.pi * 0.5) / waypoint_sep)
    thetas = np.linspace(1 * np.pi / 2, 0, num_turn_points)
    turn_poses = [[turn_radius * np.cos(theta) + turn_center[0], turn_radius * np.sin(theta) + turn_center[1], theta - (np.pi / 2)] for theta in thetas]
    poses = straight_poses + turn_poses
    return poses


def cse022_path():
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../scripts/cse022_path.pickle', 'r') as f:
        p = pickle.load(f)
    return p

def single():
    poses = [[x, 0, 0] for x in xrange(10)]
    print poses
    return poses


plans = {'circle': circle, 'left turn': left_turn, 'right turn': right_turn, 'saw': saw, 'cse022 real path': cse022_path, 'single pose': single}
plan_names = ['circle', 'left turn', 'right turn', 'saw', 'cse022 real path', 'single pose']

def generate_plan():
    print "Which plan would you like to generate? "
    for i, name in enumerate(plan_names):
        print "{} ({})".format(name, i)
    index = int(raw_input("num: "))
    if index >= len(plan_names):
        print "Wrong number. Exiting."
        exit()
    path_name = plan_names[index]
    return plans[plan_names[index]](), path_name

def generate_all_plan():
    print "Which plan would you like to generate? "
    plan_dict = {}
    for i, name in enumerate(plan_names):
        plan_dict[name] = plans[plan_names[i]]()
    return plan_dict

if __name__ == '__main__':
    rospy.init_node("controller_runner")
    configs, path_name = generate_plan()

    if type(configs) == XYHVPath:
        rospy.loginfo('022 path called')
        path = configs
    else:
        h = Header()
        h.stamp = rospy.Time.now()
        desired_speed = 2 # 0.5
        ramp_percent = 0.1
        ramp_up = np.linspace(0.0, desired_speed, int(ramp_percent * len(configs)))
        ramp_down = np.linspace(desired_speed, 0.3, int(ramp_percent * len(configs)))
        speeds = np.zeros(len(configs))
        speeds[:] = desired_speed
        print len(configs)
        speeds[0:len(ramp_up)] = ramp_up
        speeds[-len(ramp_down):] = ramp_down
        path_ = XYHVPath_log(h, [XYHV(*[config[0], config[1], config[2], speed]) for config, speed in zip(configs, speeds)], path_name)
        path = XYHVPath(h, [XYHV(*[config[0], config[1], config[2], speed]) for config, speed in zip(configs, speeds)])
    print "Sending path..."
    # controller = rospy.ServiceProxy("/controller/follow_path", FollowPath())
    controller_log = rospy.ServiceProxy("/controller/follow_path_log", FollowPath_log())
    # success = controller(path)
    success = controller_log(path_, path_name)
    print "Controller started."
 
