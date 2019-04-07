#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import visualization_msgs

class PoseMarker:
	def __init__(self):
		rospy.init_node('marker', anonymous=True)

		self.pose_sub = rospy.Subscriber('/sim_car_pose/pose', PoseStamped, self.pose_cb, queue_size=1)
		self.pose = None

		self.marker_pub = rospy.Publisher('/pose_markers/markers', Marker, queue_size=1)

		self.rate = rospy.Rate(2)
		
		id_cnt = 0
		
		while not rospy.is_shutdown():
			marker = Marker()
			marker.header.frame_id = '/map'
			marker.header.stamp = rospy.Time.now()
			marker.type = Marker.ARROW
			marker.action = Marker.ADD
			marker.pose = self.pose
			marker.scale.x = 1
			marker.scale.y = 0.05
			marker.scale.z = 0.05
			marker.color.a = 1.0
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
			marker.id = id_cnt
			id_cnt += 1
			self.marker_pub.publish(marker)
			self.rate.sleep()


	def pose_cb(self, data):
		self.pose = data.pose

if __name__ == '__main__':
    m = PoseMarker()
