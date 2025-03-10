import numpy as np
import rospy
from controller import BaseController


class PurePursuitController(BaseController):
    def __init__(self):
        super(PurePursuitController, self).__init__()

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
            # Use the pure pursuit lookahead method described in the
            # handout for determining the reference index.
            #
            # Note: this method must be computationally efficient
            # as it is running directly in the tight control loop.
            include_heading = False
            if include_heading:
                dist = np.sum((self.path[:,0:3] - pose[0:3])**2), axis=1)
            else:
                dist = np.sum((self.path[:,0:2] - pose[0:2])**2), axis=1)
            dist = np.abs(dist - self.pose_lookahead)
            indices = dist.argsort()[:-2][::-1]
            return indices[1] if indices[1] > indices[0] else indices[0]

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
        # First, compute the appropriate error.
        #
        # Then, use the pure pursuit control method to compute the
        # steering angle. Refer to the hand out and referenced
        # articles for more details about this strategy.
        pose_ref = get_reference_pose(index)
        x_ref, y_ref, V_ref = pose_ref[0], pose_ref[1], pose_ref[3]
        x, y = pose[0:2]
        theta = pose[2]
        alpha = np.arctan((y_ref-y)/(x_ref-x)) - theta
        u_steering = np.arctan(2*self.B*np.sin(alpha)/self.pose_lookahead)

        return [V_ref, u_steering]

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        pass

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            # what does self.speed use for? should we get V_ref from reference pose or here?
            self.speed = float(rospy.get_param("/pid/speed", 1.0))
            self.finish_threshold = float(rospy.get_param("/pid/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/pid/exceed_threshold", 4.0))
            # Lookahead distance from current pose to next waypoint. Different from
            # waypoint_lookahead in the other controllers, as those are distance from
            # the reference point.
            self.pose_lookahead = float(rospy.get_param("/purepursuit/pose_lookahead", 0.6))
