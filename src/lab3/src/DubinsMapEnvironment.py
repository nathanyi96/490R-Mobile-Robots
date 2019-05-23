import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from Dubins import dubins_path_planning
from MapEnvironment import MapEnvironment

class DubinsMapEnvironment(MapEnvironment):

    def __init__(self, map_data, curvature=5):
        super(DubinsMapEnvironment, self).__init__(map_data)
        self.curvature = curvature
        self.vectorized_func = self.vectorize(vectorize_compute_distances)

    def vectorize_compute_distances(self, end_config, start_config):
        curvature = np.sqrt(np.sum(((np.subtract(end_config, start_config))**2)), axis=1) / 2 # Curvature = radius = diameter/2
        _, distance = dubtin_path_planning(start_config, end_config, curvature)

    def compute_distances(self, start_config, end_configs):
        """
        Compute distance from start_config and end_configs using Dubins path
        @param start_config: tuple of start config
        @param end_configs: list of tuples of end confings
        @return numpy array of distances
        """

        # Implement here

        distances = self.vectorize_computer_distances(end_configs, start_config)

        return distances

    def compute_heuristic(self, config, goal):
        """
        Use the Dubins path length from config to goal as the heuristic distance.
        """

        # Implement here

        curvature = np.sqrt(np.sum(((np.subtract(goal, config))**2)), axis=1) / 2 # Curvature = radius = diameter/2
        _, heuristic = dubins_path_planning(config, goal, curvature)

        return heuristic

    def generate_path(self, config1, config2):
        """
        Generate a dubins path from config1 to config2
        The generated path is not guaranteed to be collision free
        Use dubins_path_planning to get a path
        return: (numpy array of [x, y, yaw], curve length)
        """

        # Implement here

        curvature = np.sqrt(np.sum(((np.subtract(config2, config1))**2)), axis=1) / 2 # Curvature = radius = diameter/2
        x_path, y_path, theta_path, path_length = dubins_path_planning(config1, config2, curvature) 
        path = np.array([x_path, y_path, theta_path])
        clen = path_length

        return path, clen
