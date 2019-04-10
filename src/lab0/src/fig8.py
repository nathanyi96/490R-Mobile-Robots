#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import angle_to_quaternion
import numpy as np
import rosbag
from geometry_msgs.msg import PoseStamped
from utils import *


heading_flag = 1
velocity_flag = 1
noise_eps = 0.01

velocity = 0.0
steering_angle = 0.0

def circle_callback(pose):
    global heading_flag, velocity_flag, steering_angle
    heading = quaternion_to_angle(pose.pose.orientation)
    thre = [steering_angle, steering_angle/5]
    print heading
    if heading_flag == 1 and heading > -thre[0] and heading < -thre[1]:
        heading_flag = -1
    if heading_flag == -1 and heading < thre[0] and heading > thre[1]:
        velocity_flag = 0

if __name__ == '__main__':
    rospy.init_node('fig8', anonymous=True)

    velocity = rospy.get_param('~velocity')
    steering_angle = rospy.get_param('~steering_angle')

    pub_nav0 = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
    pub_init = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    sub_pose = rospy.Subscriber('sim_car_pose/pose', PoseStamped, circle_callback, queue_size=1)

    

    # set initial pose
    init_x = 0.0
    init_y = 0.0
    init_heading = 0.0
    rospy.loginfo("Setting init pose: " + str((init_x, init_y, init_heading)))
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.stamp = rospy.Time.now()
    init_msg.header.frame_id = "/map"
    init_msg.pose.pose.position.x = init_x
    init_msg.pose.pose.position.y = init_y
    init_msg.pose.pose.orientation = angle_to_quaternion(init_heading)


    rospy.sleep(1.0)
    pub_init.publish(init_msg)
    rospy.sleep(0.5)

    cmdcnt = 0
    rate = rospy.Rate(20)

    # try heading 
    while not rospy.is_shutdown():
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "/map"
        cmd.header.seq = cmdcnt
        cmd.drive.steering_angle = steering_angle * heading_flag 
        cmd.drive.speed = velocity * velocity_flag
        pub_nav0.publish(cmd)
        cmdcnt += 1
        rate.sleep()

    rospy.spin()

