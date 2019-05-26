import numpy as np
from matplotlib import pyplot as plt
import IPython
class MapEnvironment(object):

    def __init__(self, map_data, stepsize=0.5):
        """
        @param map_data: 2D numpy array of map
        @param stepsize: size of a step to generate waypoints
        """
        # Obtain the boundary limits.
        # Check if file exists.
        self.map = map_data
        self.xlimit = [0, np.shape(self.map)[0]]
        self.ylimit = [0, np.shape(self.map)[1]]
        self.limit = np.array([self.xlimit, self.ylimit])
        self.maxdist = np.float('inf')
        self.stepsize = stepsize
        print()
        # Display the map.
        plt.imshow(self.map, interpolation='nearest', origin='lower')
        plt.savefig('map.png')
        print("Saved map as map.png")

    def state_validity_checker(self, configs):
        """
        @param configs: 2D array of [num_configs, dimension].
        Each row contains a configuration
        @return numpy list of boolean values. True for valid states.
        """
        if len(configs.shape) == 1:
            configs = configs.reshape(1, -1)

        #x_coordinates = configs[:,0]
        #y_coordinates = configs[:,1]

        ## Implement here
        ## 1. Check for state bounds within xlimit and ylimit
        #x_within_limits = np.where((x_coordinates > self.xlimit[0]) & (x_coordinates < self.xlimit[1]), True, False)
        #y_within_limits = np.where((y_coordinates > self.ylimit[0]) & (y_coordinates < self.ylimit[1]), True, False)
        #xy_limits_bool = np.column_stack((x_within_limits, y_within_limits), axis=1)
        #all_xy_within_limits = np.all(within_limits, axis=1) # Should have shape (num_configs,)

        ## 2. Check collision
        #locations_collisions = self.map[x_coordinates, y_coordinates]
        #check_collisions = np.where(locations_collisions == 0, True, False)
        #get_path_on_graph
        #validity = np.logical_and(all_xy_within_limits, no_collision)
        x = configs[:,0]
        y = configs[:,1]

        # here is the reason
        # simple convert to int(linear space yields double coordinate). need a better checker
        x = np.array(x, dtype=int)
        y = np.array(y, dtype=int)

        #print(configs)
        x_1 = (x < self.xlimit[1] ) * (x >= self.xlimit[0])
        y_1 = (y < self.ylimit[1] ) * (y >= self.ylimit[0])
        check1 = x_1 * y_1
        # 2. Check collision
        x = np.clip(x, self.xlimit[0], self.xlimit[1] - 1)
        y = np.clip(y, self.ylimit[0], self.ylimit[1] - 1)
        check2 = self.map[x,y] == 0 ## cuz map is an array. if use random.uniform, it's double
        validity = check1 * check2
        return validity

    def edge_validity_checker(self, config1, config2):
        """
        Checks whether the path between config 1 and config 2 is valid
        """
        config1 = np.array(config1)
        config2 = np.array(config2)
        path, length = self.generate_path(config1, config2)
        if length == 0:
            return False, 0

        valid = self.state_validity_checker(path)

        if not np.all(self.state_validity_checker(path)):
            return False, self.maxdist
        return True, length

    def compute_heuristic(self, config, goal):
        """
        Returns a heuristic distance between config and goal
        @return a float value
        """
        # Implement here

        heuristic =  np.linalg.norm(np.array(goal)[0:2] - np.array(config)[0:2]) # Essentially same as self.compute_distances??
        # np.linalg.norm(np.array(goal) - np.array(config), axis=1)
        return heuristic

    def compute_distances(self, start_config, end_configs):
        """
        Compute distance from start_config and end_configs in L2 metric.
        @param start_config: tuple of start config
        @param end_configs: list of tuples of end confings
        @return 1D  numpy array of distances
        """

        distances = np.linalg.norm(np.array(end_configs)[:,0:2] - np.array(start_config)[0:2], axis=1)

        return distances

    def generate_path(self, config1, config2):
        config1 = np.array(config1)[0:2]
        config2 = np.array(config2)[0:2]
        dist = np.linalg.norm(config2 - config1)
        if dist == 0:
            return config1, dist
        direction = (config2 - config1) / dist
        steps = dist // self.stepsize + 1

        waypoints = np.array([np.linspace(config1[i], config2[i], steps) for i in range(2)]).transpose()

        return waypoints, dist

    def get_path_on_graph(self, G, path_nodes):
        plan = []
        for node in path_nodes:
            #plan += [G.nodes[node]["config"]]
            plan += [node]
        plan = np.array(plan)

        path = []
        xs, ys, yaws = [], [], []
        for i in range(np.shape(plan)[0] - 1):
            path += [self.generate_path(plan[i], plan[i+1])[0]]

        return np.concatenate(path, axis=0)

    def shortcut(self, G, waypoints, num_trials=100):
        """
        Short cut waypoints if collision free
        @param waypoints list of node indices in the graph
        """
        print("Originally {} waypoints".format(len(waypoints)))
        # for _ in range(num_trials):

        for i in range(len(waypoints)):

            if len(waypoints) == 2:
                break

            # Implement here
            # 1. Choose two configurations
            # 2. Check for collision
            # 3. Connect them if collision free

            for j in range(len(waypoints), i, -1):
                start_node = waypoints[i]
                end_node = waypoints[j]

                valid, weight = self.edge_validity_checker(start_node, end_node)
                if valid:
                    G.add_weighted_edges([(start_node, end_node, weight)]) # Add new edge to graph for future reference
                    del waypoints[i+1:j] # Delete waypoints in between

        print("Path shortcut to {} waypoints".format(len(waypoints)))
        return waypoints


    def visualize_plan(self, G, path_nodes, start=None, goal=None, saveto=""):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plan = []
        for node in path_nodes:
            #plan += [G.nodes[node]["config"]]
            plan += [node]
        plan = np.array(plan)

        plt.clf()
        plt.imshow(self.map, interpolation='none', cmap='gray', origin='lower')

        # Comment this to hide all edges. This can take long.
        # edges = G.edges()
        # for edge in edges:
        #     config1 = G.nodes[edge[0]]["config"]
        #     config2 = G.nodes[edge[1]]["config"]
        #     x = [config1[0], config2[0]]
        #     y = [config1[1], config2[1]]
        #     plt.plot(y, x, 'grey')

        path = self.get_path_on_graph(G, path_nodes)
        plt.plot(path[:,1], path[:,0], 'y', linewidth=1)

        for vertex in G.nodes:
            #config = G.nodes[vertex]["config"]
            if vertex == start:
                plt.scatter(vertex[1], vertex[0], s=30, c='b')
            elif vertex == goal:
                # Color the goal node with green
                plt.scatter(vertex[1], vertex[0], s=30, c='g')
            else:
                plt.scatter(vertex[1], vertex[0], s=30, c='r')

        plt.tight_layout()

        if saveto != "":
            plt.savefig(saveto)
            return
        plt.show()

    def visualize_graph(self, G, start=None, goal=None, saveto=""):
        plt.clf()
        plt.imshow(self.map, interpolation='nearest', origin='lower')
        edges = G.edges()
        for edge in edges:
            #config1 = G.nodes[edge[0]]["config"]
            #config2 = G.nodes[edge[1]]["config"]
            config1 = edge[0]
            config2 = edge[1]
            path = self.generate_path(config1, config2)[0]
            #print("path debug", path)
            plt.plot(path[:,1], path[:,0], 'w')

        num_nodes = G.number_of_nodes()

        for i, vertex in enumerate(G.nodes()):
            config = vertex

            #if i == num_nodes - 2:
            if vertex == start:
                # Color the start node with blue
                plt.scatter(config[1], config[0], s=30, c='b')
            #elif i == num_nodes - 1:
            elif vertex == goal:
                # Color the goal node with green
                plt.scatter(config[1], config[0], s=30, c='g')
            else:
                plt.scatter(config[1], config[0], s=30, c='r')

        plt.tight_layout()

        if saveto != "":
            plt.savefig(saveto)
            return
        plt.ylabel('x')
        plt.xlabel('y')
        plt.show() #shape
