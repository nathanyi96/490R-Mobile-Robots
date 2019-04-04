#!/usr/bin/env python

import rospy
import threading
import signal

from std_msgs.msg import String, Float64
from std_srvs.srv import Empty

class Subscriber:
    def __init__(self):
        rospy.init_node('subscriber', anonymous=True, disable_signals=True)

        self.s_init = rospy.Subscriber('/lab0/initialpose', String, self.init_callback, queue_size=1)
        self.s_velocity = rospy.Subscriber('/lab0/velocity', Float64, self.vel_callback, queue_size=1)
        self.s_heading = rospy.Subscriber('/lab0/steering_angle', Float64, self.heading_callback, queue_size=1)

    def init_callback(self, init):
        def sub_init_pose():
            rospy.loginfo("Receiving init pose: " + init.data)

        return sub_init_pose()

    def vel_callback(self, v):
        def sub_vel():
            rospy.loginfo("Receiving velocity: " + str(v.data))
            #self.p_velocity.publish(Float64(float(v)))

        return sub_vel()

    def heading_callback(self, h):
        def sub_heading():
            rospy.loginfo("Receiving heading: " + str(h.data))
            #self.p_heading.publish(Float64(float(h)))

        return sub_heading()

if __name__ == '__main__':
    s = Subscriber()
    rospy.spin()
