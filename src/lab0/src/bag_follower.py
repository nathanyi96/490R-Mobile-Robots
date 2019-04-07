#!/usr/bin/env python
import rospy
import rosbag
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

if __name__ == '__main__':
    rospy.init_node('bag_follower', anonymous=True)
    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    teleop_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)

    bag_file = rospy.get_param("~bag_file")
    rospy.loginfo("bag file: " + str(bag_file))
    bag = rosbag.Bag(bag_file)
    
    init_msg = None
    bag_data = []
    for topic, msg, t in bag.read_messages(topics=['/initialpose', '/vesc/low_level/ackermann_cmd_mux/input/teleop']):
        if topic == '/initialpose':
            rospy.loginfo('read message on topic ' + topic)
            rospy.loginfo('msg: ' + str(msg))
            init_msg = msg
        else:
            bag_data.append(msg)

    rate = rospy.Rate(20)

    init_pub.publish(init_msg)
    for msg in bag_data:
        if not rospy.is_shutdown():
            teleop_pub.publish(msg)
            rate.sleep()
    bag.close()
