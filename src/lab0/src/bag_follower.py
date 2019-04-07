#!/usr/bin/env python
import rospy
import rosbag
from ackermann_msgs.msg import AckermannDriveStamped

if __name__ == '__main__':
    rospy.init_node('bag_follower', anonymous=True)

    bag_file = rospy.get_param("~bag_file")
    rospy.loginfo("bag file: " + str(bag_file))
    bag = rosbag.Bag(bag_file)
    bag_data = []
    for topic, msg, t in bag.read_messages(topics=['/initalpose', '/vesc/low_level/ackermann_cmd_mux/input/teleop']):
        bag_data.append(msg)
        rospy.loginfo('read message on topic ' + topic)

    teleop_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
    rate = rospy.Rate(2)
    #while not rospy.is_shutdown():
    for msg in bag_data:
        if not rospy.is_shutdown():
            teleop_pub.publish(msg)
            rate.sleep()
    bag.close()
