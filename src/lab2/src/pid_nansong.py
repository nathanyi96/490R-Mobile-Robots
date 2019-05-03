import numpy as np
import rospy

from controller import BaseController


class PIDController(BaseController):
    def __init__(self):
        super(PIDController, self).__init__()

        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose):
        '''
        get_reference_index finds the index i in the controller's path
            to compute the next control control against
        input:
            pose - current pose of the car, represented as [x, y, heading]
        output:
            i - referencence index
        '''
        with self.path_lock:
            # TODO: INSERT CODE HERE
            # Determine a strategy for selecting a reference point
            # in the path. One option is to find the nearest reference
            # point and chose the next point some distance away along
            # the path.
            #
            # Note: this method must be computationally efficient
            # as it is running directly in the tight control loop.
            include_heading = False
            if include_heading:
                dist = np.sum((self.path[:,0:3] - pose[0:3])**2), axis=1)
            else:
                dist = np.sum((self.path[:,0:2] - pose[0:2])**2), axis=1)
            # Find reference point with minimum, then return next point since robot may be ahead
            return np.argmin(dist) + 2  ## +2 to make sure the ref is ahead of current pos

    def get_control(self, pose, index):
        '''
        get_control - computes the control action given an index into the
            reference trajectory, and the current pose of the car.
            Note: the output velocity is given in the reference point.
        input:
            pose - the vehicle's current pose [x, y, heading]
            index - an integer corresponding to the reference index into the
                reference path to control against
        output:
            control - [velocity, steering angle]
        '''
        # TODO: INSERT CODE HERE
        # Compute the next control using the PD control strategy.
        # Consult the project report for details on how PD control
        # works.
        #
        # First, compute the cross track error. Then, using known
        # gains, generate the control.
        e_p = self.get_error(pose, index)
        pose_ref = self.get_reference_pose(index)
        e_theta = pose[2] - pose_ref[2]
        V = pose_ref[3]

        e_c = e_p[1] # cross error term
        e_c_der = V*np.sin(e_theta) # error derivative term

        u_steering = self.kp * e_c + self.kd * e_c_der
        return [V, u_steering]

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        with self.path_lock:
            pass

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.kp = float(rospy.get_param("/pid/kp", 0.15))
            self.kd = float(rospy.get_param("/pid/kd", 0.2))
            self.finish_threshold = float(rospy.get_param("/pid/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/pid/exceed_threshold", 4.0))
            # Average distance from the current reference pose to lookahed.
            # what does this waypoint_lookahead do? maybe useful in find reference point?
            self.waypoint_lookahead = float(rospy.get_param("/pid/waypoint_lookahead", 0.6))
