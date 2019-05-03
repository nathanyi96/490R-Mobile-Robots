#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import angle_to_quaternion
import numpy as np
import rosbag
from geometry_msgs.msg import PoseStamped
from utils import *
from sensor_msgs.msg import LaserScan

steering_angle = 0.0
class SafetyScan:

    def __init__(self):
        self.stop_flag = False
        self.pub_safety = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)

        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.stop_flag:
                cmd = AckermannDriveStamped()
                cmd.header.stamp = rospy.Time.now()
                cmd.header.frame_id = "/map"
                cmd.drive.speed = 0
                self.pub_safety.publish(cmd)
            self.rate.sleep()

    def laser_callback(self, msg):
        # print str(len(msg.ranges))
        # print str((msg.angle_max - msg.angle_min) / msg.angle_increment)
        ranges = np.array(msg.ranges)
        ranges = ranges[np.where(~np.isnan(ranges))]
        ranges = ranges[ranges < msg.range_max]
        ranges = ranges[ranges > msg.range_min]
        close = (ranges < 0.5).sum()
        far = (ranges > 1.5).sum()
        print 'minimum scan: {}, {}'.format(ranges.min(), close)
        if close > 5: # threshold
            # rospy.loginfo('< min')
            print 'check collision'
            print rospy.Time.now()
            self.stop_flag = True
        elif far > 20:
            self.stop_flag = False
if __name__ == '__main__':
    rospy.init_node('scan', anonymous=True)
    s = SafetyScan()
