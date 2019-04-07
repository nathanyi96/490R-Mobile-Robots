#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from utils import angle_to_quaternion

class Subscriber:
    def __init__(self):
        rospy.init_node('subscriber', anonymous=True)

        self.pub_nav0 = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
        self.pub_init = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        self.sub_init = rospy.Subscriber('/lab0/initpose', String, self.init_callback, queue_size=1)
        self.sub_velocity = rospy.Subscriber('/lab0/velocity', Float64, self.vel_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/lab0/heading', Float64, self.heading_callback, queue_size=1)
        
        self.srv_reset = rospy.Service('subscriber/reset', Empty, self.reset)
        
        self._velocity = 0
        self._heading = 0
        self._cmdcnt = 0

        self.rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            # TODO: publish to VESC
            cmd = AckermannDriveStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "/map"
            cmd.header.seq = self._cmdcnt
            cmd.drive.steering_angle = self._heading
            cmd.drive.speed = self._velocity
            self.pub_nav0.publish(cmd)
            self._cmdcnt += 1
            self.rate.sleep()

    def init_callback(self, init):
        def sub_init_pose():
            rospy.loginfo("Receiving init pose: " + init.data)
            # TODO: parse initial pose and publish to /initialpose
            init_data = init.data.split(',')
            init_msg = PoseWithCovarianceStamped()
            init_msg.header.stamp = rospy.Time.now()
            init_msg.header.frame_id = "/map"
            init_msg.pose.pose.position.x = float(init_data[0])
            init_msg.pose.pose.position.y = float(init_data[1])
            init_msg.pose.pose.orientation = angle_to_quaternion(float(init_data[2]))
            # init_msg.pose.covariance = # TODO: check if this line is needed
            
            self.pub_init.publish(init_msg)
        return sub_init_pose()

    def vel_callback(self, v):
        def sub_vel():
            rospy.loginfo("Receiving velocity: " + str(v.data))
            self._velocity = v.data
        return sub_vel()

    def heading_callback(self, h):
        def sub_heading():
            rospy.loginfo("Receiving heading: " + str(h.data))
            self._heading = h.data
        return sub_heading()

    def reset(self, req):
        rospy.loginfo('Reset velocity and heading to 0.')
        self._velocity = 0
        self._heading = 0
        return EmptyResponse()


if __name__ == '__main__':
    s = Subscriber()
    rospy.spin()
