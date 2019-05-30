import numpy as np
import math
import random
import IPython

def informed_sample(cMax, cMin, start, end, num_sample):
    xCenter = (start + end) / 2.0
    axis = end - start
    theta = np.arctan2(axis[1], axis[0])
    rotmat = np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta),np.cos(theta), 0], [0, 0, 1]])

    r = [cMax / 2.0, #
         math.sqrt(cMax**2 - cMin**2) / 2.0,
         0]
    L = np.diag(r)
    rnds = []
    ellipse = [[xCenter[1], xCenter[0]], cMax, math.sqrt(cMax**2 - cMin**2), 90 - theta * 180 / np.pi]
    for _ in range(num_sample):
        xBall = sampleUnitBall()
        rnd = rotmat.dot(L).dot(xBall) + xCenter
        rnd = (rnd[0, 0], rnd[1, 0], rnd[2, 0])
        rnds.append(rnd)
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


    ell = Ellipse(xy=np.array([0, 0]), width=3, height=1.0, angle=45)

    fig, ax = plt.subplots(subplot_kw={'aspect': 'equal'})
    ax.add_artist(ell)
    ell.set_clip_box(ax.bbox)
    ell.set_alpha(0.4)
    ell.set_facecolor(np.random.rand(3))

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

    x, y = np.array([-1, -1]), np.array([1, 1])

    points, ellipse = informed_sample(3, 2*np.sqrt(2), x, y, 1000)
    points = np.array(points)
    ax.scatter(points[:,0], points[:,1], color='g')
    ax.scatter([x[0], y[0]], [x[1], y[1]], color='r')

    plt.show()
