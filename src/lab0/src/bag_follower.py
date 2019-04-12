#!/usr/bin/env python
import rospy
import rosbag
from utils import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

import time
if __name__ == '__main__':
    end = time.time()
    rospy.init_node('bag_follower', anonymous=True)
    rospy.loginfo("time = " + str(rospy.Time.now()))

    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    teleop_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=100)

    bag_file = rospy.get_param("~bag_file")
    rospy.loginfo("bag file: " + str(bag_file))
    bag = rosbag.Bag(bag_file)

    backward =rospy.get_param("~backward")
    rospy.loginfo('backward = ' + str(backward))
    
    topic_set = set()
    init_msg = None
    bag_data = []
    for topic, msg, t in bag.read_messages(topics=['/initialpose', '/vesc/low_level/ackermann_cmd_mux/input/teleop']):
        if topic not in topic_set:
            topic_set.add(topic)
        if topic == '/initialpose':
            init_msg = msg
        else:
            bag_data.append(msg)
    rospy.sleep(1.0)
    rospy.loginfo('topic set: ' + str(topic_set))
    if not init_msg:  # if initial pose not recorded, set to x=0, y=0, heading=0
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.stamp = rospy.Time.now()
        init_msg.header.frame_id = "/map"
        init_msg.pose.pose.position.x = 0.0
        init_msg.pose.pose.position.y = 0.0
        init_msg.pose.pose.orientation = angle_to_quaternion(0.0)
    rospy.loginfo('Initial pose: \n' + str(init_msg))
    init_pub.publish(init_msg)
    rate = rospy.Rate(23)

    cnt = 0
    for msg in bag_data:
        if backward:
            msg.drive.speed = -msg.drive.speed
        if cnt == 0:
            rospy.loginfo(msg)
        teleop_pub.publish(msg)
        cnt += 1
        if cnt % 100 == 0:
            print ("time = " + str(rospy.Time.now()))
        rate.sleep()
    bag.close()
    print 'time spent', time.time() - end
