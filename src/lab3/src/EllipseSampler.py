import numpy as np
import math
import random
import IPython
np.random.seed(3)

def informed_sample(cMax, cMin, start, end, num_sample):
    xCenter = ((start + end) / 2.0)
    xCenter[-1] = 0
    axis = end - start
    theta = np.arctan2(axis[1], axis[0])
    rotmat = np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta),np.cos(theta), 0], [0, 0, 1]])

    r = [cMax / 2.0, # a 
         math.sqrt(cMax**2 - cMin**2) / 2.0, # b
         0]
    L = np.diag(r)
    rnds = []
    ellipse = [[xCenter[0], xCenter[1]], cMax, math.sqrt(cMax**2 - cMin**2), theta * 180 / np.pi]
    # ellipse = [[xCenter[1], xCenter[0]], cMax, math.sqrt(cMax**2 - cMin**2), 90 - theta * 180 / np.pi]
    for _ in range(num_sample):
        xBall = sampleUnitBall()
        rnd = rotmat.dot(L.dot(xBall)) + xCenter.reshape([3,1])
        rnd = (rnd[0, 0], rnd[1, 0], rnd[2, 0])
        rnds.append(rnd)
    # IPython.embed()
    return rnds, ellipse

def sampleUnitBall():
    r = np.sqrt(random.random())
    angle = np.random.uniform(0, 2 * math.pi)
    sample = (r * math.cos(angle),
              r * math.sin(angle))
    return np.array([[sample[0]],[sample[1]], [0]])

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse
    c_2 = np.random.rand() + 0.2
    a_2 = c_2 + 0.1
    ang = np.random.randn() * np.pi
    xCenter = np.random.randn(3, 1)
    c = c_2 * np.cos(ang)
    s = c_2 * np.sin(ang)
    x, y = np.array([[-c], [-s], [0]]) + xCenter, np.array([[c], [s], [0]]) + xCenter
    points, ellipse = informed_sample(2 * a_2, 2 * c_2, x, y, 1000)

    ell = Ellipse(*ellipse)
    fig, ax = plt.subplots(subplot_kw={'aspect': 'equal'})
    ax.add_artist(ell)
    ell.set_clip_box(ax.bbox)
    ell.set_alpha(0.5)
    ell.set_facecolor((1,0,0))

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    points = np.array(points)
    ax.scatter(points[:,0], points[:,1], color='g')
    ax.scatter([x[0], y[0]], [x[1], y[1]], color='r')

    plt.show()
