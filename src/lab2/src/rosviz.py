import matplotlib.cm as cm
import matplotlib.colors as colors
import rospy
import numpy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

_traj_pub = rospy.Publisher("/trajs_array", MarkerArray, queue_size=100)


def viz_paths_cmap(poses, costs, ns="paths", cmap='plasma', scale=.03):
    """
    Visualize poses and costs in rviz.
    Call this function from ModelPredictiveControl.get_control for visualization.
    """
    max_c = numpy.max(costs)
    min_c = numpy.min(costs)

    norm = colors.Normalize(vmin=min_c, vmax=max_c)

    cmap = cm.get_cmap(name=cmap)

    def colorfn(cost):
        r, g, b, a = 0.0, 0.0, 0.0, 1.0
        col = cmap(norm(cost))
        r, g, b = col[0], col[1], col[2]
        if len(col) > 3:
            a = col[3]
        return r, g, b, a

    return viz_paths(poses, costs, colorfn, ns, scale)


def viz_paths(poses, costs, colorfn, ns="paths", scale=.03):
    """
        poses should be an array of trajectories to plot in rviz
        costs should have the same dimensionality as poses.size()[0]
        colorfn maps a point to an rgb tuple of colors
    """
    assert poses.shape[0] == costs.shape[0]

    markers = MarkerArray()

    for i, (traj, cost) in enumerate(zip(poses, costs)):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = i
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.pose.position.x = 0
        m.pose.position.y = 0
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.color.r, m.color.g, m.color.b, m.color.a = colorfn(cost)

        for t in traj:
            p = Point()
            p.x, p.y = t[0], t[1]
            m.points.append(p)

        markers.markers.append(m)


    _traj_pub.publish(markers)
