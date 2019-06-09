import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import dubins
from MapEnvironment import MapEnvironment

class DubinsMapEnvironment(MapEnvironment):

    def __init__(self, map_data, curvature=5):
        super(DubinsMapEnvironment, self).__init__(map_data)
        self.curvature = curvature

    def compute_distances(self, start_config, end_configs):
        """
        Compute distance from start_config and end_configs using Dubins path
        @param start_config: tuple of start config
        @param end_configs: list of tuples of end confings
        @return numpy array of distances
        """
        # Implement here
        turning_radius = 1.0 / self.curvature
        num = len(end_configs)
        distances = np.empty(num)
        for i in range(num):
            path = dubins.shortest_path(start_config, end_configs[i], turning_radius)
            distances[i] = path.path_length()
        return distances

    def compute_heuristic(self, config, goal):
        """
        Use the Dubins path length from config to goal as the heuristic distance.
        """

        # Implement here
        path = dubins.shortest_path(config, goal, 1.0 / self.curvature)
        return path.path_length()

    def generate_path(self, config1, config2):
        """
        Generate a dubins path from config1 to config2
        The generated path is not guaranteed to be collision free
        Use dubins_path_planning to get a path
        return: (numpy array of [x, y, yaw], curve length)
        """
        path = dubins.shortest_path(config1, config2, 1.0 / self.curvature)
        configs, _ = path.sample_many(0.2)
        return np.array(configs), path.path_length()
