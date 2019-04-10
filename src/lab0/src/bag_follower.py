#!/usr/bin/env python
import rospy
import rosbag
from utils import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
# /vesc/high_level/ackermann_cmd_mux/input/nav_0
if __name__ == '__main__':
    rospy.init_node('bag_follower', anonymous=True)

    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    teleop_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

    bag_file = rospy.get_param("~bag_file")
    rospy.loginfo("bag file: " + str(bag_file))
    bag = rosbag.Bag(bag_file)

    backward =rospy.get_param("~backward")
    rospy.loginfo('backward = ' + str(backward))
    
    init_msg = None
    bag_data = []
    for topic, msg, t in bag.read_messages(topics=['/initialpose', '/vesc/low_level/ackermann_cmd_mux/input/teleop']):
        if topic == '/initialpose':
            # rospy.loginfo('read message on topic ' + topic)
            # rospy.loginfo('msg: ' + str(msg))
            init_msg = msg
        else:
            bag_data.append(msg)

    rospy.sleep(1.0)
    if not init_msg:  # if initial pose not recorded, set to x=0, y=0, heading=0
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.stamp = rospy.Time.now()
        init_msg.header.frame_id = "/map"
        init_msg.pose.pose.position.x = 0.0
        init_msg.pose.pose.position.y = 0.0
        init_msg.pose.pose.orientation = angle_to_quaternion(0.0)
    rospy.loginfo('Initial pose: \n' + str(init_msg))
    init_pub.publish(init_msg)
    rate = rospy.Rate(20)

    cnt = 0
    for msg in bag_data:
        if not rospy.is_shutdown():
            if backward:
                msg.drive.speed = -msg.drive.speed
                # msg.drive.steering_angle = -msg.drive.steering_angle
            if cnt == 0:
                rospy.loginfo(msg)
            teleop_pub.publish(msg)
            cnt += 1
            rate.sleep()
    bag.close()
