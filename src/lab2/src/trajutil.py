import time
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from scipy.spatial.distance import directed_hausdorff
import argparse

_trajlib_path = os.path.dirname(os.path.realpath(__file__)) + '/../traj_lib/depth_{}_branch_{}_size_{}.npy'
_MIN_DELTA = -0.34
_MAX_DELTA = 0.34
_CAR_LENGTH = 0.33
_VELOCITY = 2.0
_DT = 0.2

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
            newfile = os.path.dirname(os.path.realpath(__file__)) + '/../traj_lib/depth_{}_branch_{}_size_{}.npy'.format(t, num_branches, len(S))
            print 'generate trajlib {}'.format(newfile)
            np.save(newfile, ctrls)


def apply_kinematics(cur_x, control, dt):
    x = cur_x[:,0]
    y = cur_x[:,1]
    theta = cur_x[:,2]
    velocity = control[:,0]
    delta = control[:,1]

    theta_dot = theta + (velocity / _CAR_LENGTH * np.tan(delta)) * dt
    x_dot = x + (_CAR_LENGTH / (np.tan(delta)+1e-12) * (np.sin(theta_dot) - np.sin(theta))) 
    y_dot = y + (_CAR_LENGTH / (np.tan(delta)+1e-12) * (-np.cos(theta_dot) + np.cos(theta)))
    straight_idx = (np.absolute(delta) < 1e-8)
    x_dot[straight_idx] = x[straight_idx] + ((np.cos(theta))[straight_idx])*velocity[straight_idx]*dt
    y_dot[straight_idx] = y[straight_idx] + ((np.sin(theta))[straight_idx])*velocity[straight_idx]*dt
    return (x_dot, y_dot, theta_dot)



def visualize_traj(K, T, num_branches):  # must be called after get_control_trajectories()
    filename = _trajlib_path.format(T, num_branches, K)
    if not os.path.exists(filename):
        print "file not exist: {}".format(filename)
        print "Run python trajutil generate <steps> <num_branches> to generate trajectories library of size in range(60, 150, 10)."
        return
    trajs = np.load(filename)
    fig, ax = plt.subplots()
    rollouts = np.zeros((K, T+1, 3))
    rollouts[:,0,0] = 0
    rollouts[:,0,1] = 0
    rollouts[:,0,2] = np.pi/2
    trajs[:,:,0] = _VELOCITY
    for i in xrange(T):
        rollouts[:,i+1,:] = np.array(apply_kinematics(rollouts[:,i,:], trajs[:,i,:], _DT)).T
    for i in xrange(K):
        plot_path(ax, rollouts[i])
    plt.show()


def plot_path(ax, poses):
    '''
    poses: (T x 3) ndarray, where each pose is (x, y, theta), theta in [-pi, pi]
    '''
    Path = mpath.Path

    for i in xrange(poses.shape[0]):
      ax.arrow(poses[i,0], poses[i,1], 0.05*np.cos(poses[i,2]), 0.05*np.sin(poses[i,2]), width=0.003)

    path_verts = [poses[0,:2]]
    path_codes = [Path.MOVETO]
    for i in xrange(1, poses.shape[0]):
      heading_diff = abs(poses[i-1,2] - poses[i,2])
      if heading_diff < 1e-8 or abs(heading_diff-2*np.pi) < 1e-8:
        path_verts.append(poses[i,:2])
        path_codes.append(Path.LINETO)
      else:
        ct1, ct2 = get_control_points(poses[i-1], poses[i])
        path_verts.extend((ct1, ct2, poses[i,:2]))
        path_codes.extend((Path.CURVE4,)*3)
    #return path_verts, path_codes
    pp1 = mpatches.PathPatch(
      Path(path_verts,
           path_codes),
      fc="none", transform=ax.transData)
    ax.add_patch(pp1)
    ax.plot([0], [0], "ro")
    ax.set_title('The red point is the initial location')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-0.1, 3)
    ax.set_aspect(aspect='equal', adjustable='box')


def get_control_points(pose1, pose2):  # helper method for visualization
    phi = pose1[2]-pose2[2]
    if phi > np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    L = np.sqrt((pose2[0]-pose1[0])**2 + (pose2[1]-pose1[1])**2)
    R = L / (2 * np.sin(abs(phi/2)))
    ofs_angle_sign = np.pi/2 if phi > 0 else -np.pi/2
    ofs_angle = (pose2[2] + phi/2) + ofs_angle_sign
    while ofs_angle > np.pi: ofs_angle -= 2*np.pi
    while ofs_angle < -np.pi: ofs_angle += 2*np.pi
    dir1 = pose1[2]-ofs_angle_sign
    dir2 = pose2[2]-ofs_angle_sign
    ofs_pos = np.array((R*np.cos(dir1), R*np.sin(dir1))) + pose1[:2]
    ofs_pos2 = np.array((R*np.cos(dir2), R*np.sin(dir2))) + pose2[:2]
    #print(R)
    #print(ofs_angle, np.pi/4)
    #print(dir1, dir2)
    #print(ofs_pos, ofs_pos2)
    assert np.linalg.norm(ofs_pos - ofs_pos2) < 1e-8
    # get control points of simple arc
    P0 = np.array( (np.cos(phi/2), np.sin(phi/2)) )
    #P3 = np.array( (P0[0], -P0[1]) )
    P1 = np.array( ((4-P0[0]) / 3, (1-P0[0]) * (3-P0[0])/(3 * P0[1])) )
    P2 = np.array( (P1[0], -P1[1]) )
    # apply transformation to control points
    c, s = np.cos(ofs_angle), np.sin(ofs_angle)
    rot_mat = np.array([[c, -s], [s, c]])
    P0 = np.matmul(rot_mat, R*P0) + ofs_pos
    P1 = np.matmul(rot_mat, R*P1) + ofs_pos
    P2 = np.matmul(rot_mat, R*P2) + ofs_pos
    #P3 = np.matmul(rot_mat, R*P3) + ofs_pos
    #print(P0, pose1)
    #print(P3, pose2)
    assert np.linalg.norm(P0 - pose1[:2]) < 1e-8
    #assert np.linalg.norm(P3 - pose2[:2]) < 1e-8
    return P1, P2


#precompute_traj_lib(t=3, num_branches=21, min_delta=-0.34, max_delta=0.34)
if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='trajutil')
    
    subparsers = parser.add_subparsers(help='subcommands')
    parser_viz = subparsers.add_parser('visualize', help='visualize generated trajectories')
    parser_viz.add_argument('steps', type=int, help='number of time steps of controls specified by a trajectory')
    parser_viz.add_argument('branches', type=int, help='number of branches to explore at each time step for the dense set D')
    parser_viz.add_argument('size', type=int, help='number of trajectories to choose from the dense set D')
    parser_viz.set_defaults(func=visualize_traj)

    parser_gen = subparsers.add_parser('generate', help='precompute a trajectory library, with size from range(60, 150, 10)')
    parser_gen.add_argument('steps', type=int, help='number of time steps of controls specified by a trajectory')
    parser_gen.add_argument('branches', type=int, help='number of branches to explore at each time step for the dense set D')
    parser_gen.set_defaults(func=precompute_traj_lib)

    args = parser.parse_args()
    print args
    if args.func == precompute_traj_lib:
        precompute_traj_lib(args.steps, args.branches, _MIN_DELTA, _MAX_DELTA)
    elif args.func == visualize_traj:
        visualize_traj(args.size, args.steps, args.branches)

