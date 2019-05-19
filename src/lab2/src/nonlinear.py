import numpy as np
import rospy

from controller import BaseController


# Uses Proportional-Differential Control from
# https://www.a1k0n.net/2018/11/13/fast-line-following.html
class NonLinearController(BaseController):
    def __init__(self):
        super(NonLinearController, self).__init__()

        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose):

        with self.path_lock:
            dist = np.sqrt(np.sum(((self.path[:,0:2] - pose[0:2])**2), axis=1))
            idx = np.argmin(dist)
            return (idx if (idx == len(self.path)-1) else idx+1)

    def get_control(self, pose, index):
        pose_ref = self.get_reference_pose(index)

        k_1 = self.k1
        k_2 = self.k2
        velocity = pose_ref[3]
        e_ct = self.get_error(pose, index)[1]

        theta_err = pose[2] - pose_ref[2]
        theta_err = self.minimized_angle(theta_err)

        y = -k_1 * e_ct * velocity * np.sin(theta_err) - k_2 * theta_err**2
        x = theta_err * velocity / self.B
        steering_angle = np.arctan(y/x) if abs(x) > 1e-8 else np.arctan(-k_1*e_ct*self.B)
        rospy.loginfo('v:{}, delta:{}, theta_err: {}'.format(velocity, steering_angle, theta_err))
        steering_angle = np.clip(steering_angle, -0.34, 0.34) 
        return np.array([velocity, steering_angle])


    def reset_state(self):
        with self.path_lock:
            pass

    def reset_params(self):
        with self.path_lock:
            self.k1 = float(rospy.get_param("/lyapunov/k1", 5.0))
            self.k2 = float(rospy.get_param("/lyapunov/k2", 10.0))
            self.finish_threshold = float(rospy.get_param("/lyapunov/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/lyapunov/exceed_threshold", 4.0))
