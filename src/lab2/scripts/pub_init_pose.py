#!/usr/bin/env python
import rospy
import rosbag
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
import tf.transformations
from geometry_msgs.msg import Quaternion
import time
import sys

def angle_to_quaternion(angle):
    ''' Convert an angle in radians into a quaternion message.
    In:
        angle (float) -- The yaw angle in radians
    '''
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == '__main__':
    x, y, theta = sys.argv[1:4]
    end = time.time()
    rospy.init_node('init_pose_publisher', anonymous=True)
    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(0.5)
    init_msg = PoseWithCovarianceStamped()
    init_msg.header.stamp = rospy.Time.now()
    init_msg.header.frame_id = "/map"
    init_msg.pose.pose.position.x = float(x)
    init_msg.pose.pose.position.y = float(y)
    init_msg.pose.pose.orientation = angle_to_quaternion(float(theta))
    init_pub.publish(init_msg)
