import time
import os
import sys
import numpy as np
from scipy.spatial.distance import directed_hausdorff

def precompute_traj_lib(t, num_branches, min_delta, max_delta):
    # generate dense set of paths D
    time_start = time.time()
    deltas = np.linspace(min_delta, max_delta, num=num_branches, endpoint=True)
    D_size = num_branches**t

    grid = np.meshgrid(*([deltas] * t), indexing='ij')
    D = np.stack([x.flatten() for x in grid], axis=-1)
    D = np.concatenate((D, np.zeros([1, int(t)]))) # straight
    print 'dense generation time:', time.time()-time_start
    dcache = {}  # dcache[(p1, p2)] = d(p1, p2), where p1 \in D, p2 \in S

    K = range(60, 150, 10)
    time_start = time.time()
    # contruct subset S with (hopefully) maximum coverage
    S = set((D_size,))  # add default to S
    for i in xrange(1, max(K)):
        # find path that (might) makes maximum dispersion after adding it to S
        max_mindist = -1000000
        new_path = None
        for path in xrange(D_size):
            if path not in S:
                min_dist = 1000000000
                for path_S in S:
                    # find hausdorff distance in control space. TODO: maybe convert to state space
                    if (path, path_S) in dcache:
                        dist = dcache[(path, path_S)]
                    else:
                        p1 = D[[path]]
                        p2 = D[[path_S]]
                        dist = max(directed_hausdorff(p1, p2)[0], directed_hausdorff(p2, p1)[0])
                        dcache[(path, path_S)] = dist
                    min_dist = min(min_dist, dist)
                assert min_dist < 1000000000
                if min_dist > max_mindist:
                    max_mindist = min_dist
                    new_path = path
        assert new_path != None
        S.add(new_path)
        if len(S) in K:
            ctrls = np.zeros((len(S), t, 2))
            cnt = 0
            for path in S:
                ctrls[cnt,:,1] = D[path,:]
                cnt += 1
            print 'traj selection time:', time.time()-time_start
            np.save(os.path.dirname(os.path.realpath(__file__)) + '/../traj_lib/depth_{}_branch_{}_size_{}.npy'.format(t, num_branches, len(S)), ctrls)

#precompute_traj_lib(t=8, num_branches=7, min_delta=-0.34, max_delta=0.34)


def apply_kinematics(self, cur_x, control, dt):
    x = cur_x[:,0]
    y = cur_x[:,1]
    theta = cur_x[:,2]
    velocity = control[:,0]
    delta = control[:,1]

    theta_dot = theta + (velocity / self.B * np.tan(delta)) * dt
    x_dot = x + (self.B / (np.tan(delta)+1e-12) * (np.sin(theta_dot) - np.sin(theta))) 
    y_dot = y + (self.B / (np.tan(delta)+1e-12) * (-np.cos(theta_dot) + np.cos(theta)))
    straight_idx = (np.absolute(delta) < 1e-8)
    x_dot[straight_idx] = x[straight_idx] + ((np.cos(theta))[straight_idx])*velocity[straight_idx]*dt
    y_dot[straight_idx] = y[straight_idx] + ((np.sin(theta))[straight_idx])*velocity[straight_idx]*dt


if __name__ == '__main__':
    assert len(sys.argv) >= 2
    cmds = {
        'generate': precompute_traj_lib
        'visualize': visualize
    }
    subcmd = argv
