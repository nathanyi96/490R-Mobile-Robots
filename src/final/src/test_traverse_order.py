import numpy as np
import matplotlib.pyplot as plt
import PathManagerNode


size = 5
theta = np.linspace(0, 2*np.pi, size, endpoint=False) + np.random.normal(0, 0.1, size=size)
r = np.random.normal(10, 3, size=size)

points = np.c_[r * np.cos(theta), r * np.sin(theta)]
start = (2.0, 0.0)
ordered_waypoints = PathManagerNode._get_roundtrip_waypoints(start, points)

plt.plot(points[:, 0], points[:, 1], 'o')
plt.plot(start[0], start[1], 'ro')
plt.plot(ordered_waypoints[:, 0], ordered_waypoints[:, 1])
plt.gca().set_aspect('equal')
plt.show()


