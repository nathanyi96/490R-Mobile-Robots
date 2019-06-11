from lab2.srv import FollowPath, FollowPath_log, StampedFollowPath, SignalPathComplete
import rospy
import threading

rospy.init_node('planner', anonymous=True)    
notifyComplete = rospy.ServiceProxy("path_manager/path_complete", SignalPathComplete)
notifyComplete('main')
