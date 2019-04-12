#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import angle_to_quaternion
import numpy as np
import rosbag
from geometry_msgs.msg import PoseStamped
from utils import *

if __name__ == '__main__':
    rospy.init_node('fig8', anonymous=True)

    tires_dist = 0.335 # meters
    velocity = rospy.get_param('~velocity')  # meters/sec
    steering_angle = rospy.get_param('~steering_angle')  # rad
    max_sa = 0.34
    if steering_angle > max_sa:
        steering_angle = max_sa
    if steering_angle < -max_sa:
        steering_angle = -max_sa

    pub_nav0 = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
    pub_init = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    

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

    cmdcnt = 0
    rate = rospy.Rate(20)

    phase = 1
    period = 2 * np.pi / (np.tan(steering_angle) * velocity / tires_dist)
    print "period =", period, "secs"
    period = rospy.Duration(period)
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        if t - start_time > period:
            if phase < 2:
                phase = 2
                steering_angle = -steering_angle
                start_time = t
            else: break
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "/map"
        cmd.header.seq = cmdcnt
        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = velocity
        pub_nav0.publish(cmd)
        cmdcnt += 1
        rate.sleep()

