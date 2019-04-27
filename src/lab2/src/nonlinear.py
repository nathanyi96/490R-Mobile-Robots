import numpy as np
import rospy

from controller import BaseController


# Uses Proportional-Differential Control from
# https://www.a1k0n.net/2018/11/13/fast-line-following.html
class NonLinearController(BaseController):
    def __init__(self):
        super(NonLinearController, self).__init__()

    def get_reference_index(self, pose):
        assert False, "Complete this function"

    def get_control(self, pose, index):
        assert False, "Complete this function"

    def reset_state(self):
        with self.path_lock:
            assert False, "Complete this function"

    def reset_params(self):
        with self.path_lock:
            assert False, "Complete this function"
