import EllipseSampler as isam
import numpy as np
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
ax.scatter([x[0], y[0]], [x[1], y[1]], color='r')

for i in range(1000):
    a, b, h = isam.informed_sample(3, 2*np.sqrt(2), x, y)
    ax.scatter(a, b, color='g')

plt.show()
