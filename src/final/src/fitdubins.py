"""
A utility for computing and visualizing expected Dubins path from a set of way points.

author: Chi-Heng Hung (chh56@cs.washington.edu)
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import dubins


def generate_waypoints(size, ax=None):
    """ Generates a set of round-trip way points.
    :param size: number of way points
    :param ax: axes to plot way points and lines connecting points. Plot when not None
    :return: ndarray of shape size+1 x 2 representing xy-coordinates of way points,
             the first and last points are identical.
    """
    theta = (2 * np.pi * np.linspace(0, 1, size + 1)) + np.random.normal(0, 0.1, size=size + 1)
    theta = np.sort(theta)
    r = np.random.normal(loc=10, scale=3, size=size + 1)
    theta[-1] = theta[0] + 2 * np.pi
    r[-1] = r[0]
    y = np.c_[r * np.cos(theta), r * np.sin(theta)]
    y[-1] = y[0]
    if ax is not None:
        ax.plot(y[:, 0], y[:, 1], 'o', label='waypoints')
        ax.plot(y[:, 0], y[:, 1])
    return y


def fit_heading_spline(points, ax=None):
    """
    Compute heading at each location by taking tangent vector of a fitted cubic
    spline, given a list of way points.
    :param points: n x 2 ndarray with the first and last entry being identical
    :param ax: pyplot Axes. If given, plot the spline curve.
    :return: ndarray with headings at corresponding way points
    """
    seg_dist = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points_ts = np.empty(points.shape[0])
    np.cumsum(seg_dist, out=points_ts[1:])
    points_ts[0] = 0.0
    points_ts /= (points_ts[-1] / (points.shape[0]))
    # points_ts = np.arange(points.shape[0], dtype=np.float)
    spline = CubicSpline(points_ts, points, bc_type='periodic')
    if ax is not None:
        ts = np.linspace(0, points.shape[0], 500)
        sppts = spline(ts)
        ax.plot(sppts[:, 0], sppts[:, 1], label='spline', color="green")
    dxys = spline(points_ts, 1)
    return np.arctan2(dxys[:, 1], dxys[:, 0])


def fit_heading_random(points):
    """
    Compute heading at each location by taking random directions.
    :param points: n x 2 ndarray representing the locations, with the first and last entry being identical
    :return: ndarray with headings at corresponding way points
    """
    headings = np.random.uniform(0, 2*np.pi, size=points.shape[0])
    headings[-1] = headings[0]
    return headings


def fit_heading_straight(points):
    """
    Compute heading at each location by taking vectors pointing from one location to another.
    :param points: n x 2 ndarray representing the locations, with the first and last entry being identical
    :return: ndarray with headings at corresponding way points
    """
    dir = np.diff(points, axis=0)
    headings = np.empty(points.shape[0])
    headings[:-1] = np.arctan2(dir[:, 1], dir[:, 0])
    headings[-1] = headings[0]
    return headings


def visualize_dubins(waypoints, headings, turning_radius, ax, label='dubins'):
    """
    Plot Dubins curves that traverse a list of poses in order.
    :param waypoints: n x 2 ndarray representing the locations, with the first and last entry being identical
    :param headings: (n,) ndarray representing headings at each location
    :param turning_radius: the (maximum) turning radius of the Dubins car
    :param ax: pyplot Axes to plot on.
    :param label: a optional label for the plotted curve. default to 'dubins'
    """
    dubins_seg = []
    for i in range(waypoints.shape[0]-1):
        p1 = (waypoints[i, 0], waypoints[i, 1], headings[i])
        p2 = (waypoints[i + 1, 0], waypoints[i + 1, 1], headings[i + 1])
        dpath = dubins.shortest_path(p1, p2, turning_radius)
        points, _ = dpath.sample_many(0.05)
        points = np.array(points)
        dubins_seg.append(points)
    dubins_path = np.concatenate(dubins_seg, axis=0)
    ax.plot(dubins_path[:, 0], dubins_path[:, 1], 'r', label=label)
    return dubins_path


def _run_example():
    plt.figure(Vfigsize=(6.5, 4))
    ax = plt.gca()
    waypoints = generate_waypoints(10, ax)
    headings = fit_heading_spline(waypoints,ax)
    # headings = fit_heading_straight(waypoints)
    visualize_dubins(waypoints, headings, 1, ax)
    ax.set_aspect('equal')
    ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
    plt.show()


if __name__ == '__main__':
    # np.random.seed(105)
    _run_example()
