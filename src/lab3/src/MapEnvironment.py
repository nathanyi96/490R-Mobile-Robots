import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse
import IPython
#matplotlib.use('Agg')
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

        ## 2. Check collision
        #locations_collisions = self.map[x_coordinates, y_coordinates]
        #check_collisions = np.where(locations_collisions == 0, True, False)
        #get_path_on_graph
        #validity = np.logical_and(all_xy_within_limits, no_collision)
        x = configs[:,0]
        y = configs[:,1]

        # here is the reason
        # simple convert to int(linear space yields double coordinate). need a better checker
        x = np.array(np.round(x), dtype=int)
        y = np.array(np.round(y), dtype=int)

        #print(configs)
        x_1 = (x < self.xlimit[1] ) * (x >= self.xlimit[0])
        y_1 = (y < self.ylimit[1] ) * (y >= self.ylimit[0])
        check1 = x_1 * y_1
        # 2. Check collision
        x = np.clip(x, self.xlimit[0], self.xlimit[1] - 1)
        y = np.clip(y, self.ylimit[0], self.ylimit[1] - 1)
        check2 = (self.map[x,y] == 0)
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
        @param config: tuple or ndarray of start config
        @param goal: list of tuples, or 2D ndarray, of end confings
        @return a float value
        """
        # Implement here
        heuristic =  np.linalg.norm(np.array(goal) - np.array(config))
        return heuristic

    def compute_distances(self, start_config, end_configs):
        """
        Compute distance from start_config and end_configs in L2 metric.
        @param start_config: tuple of start config
        @param end_configs: list of tuples of end confings
        @return 1D  numpy array of distances
        """
        distances = np.linalg.norm(np.array(end_configs) - np.array(start_config), axis=1)
        return distances

    def compute_path_length(self, path):
        dist = 0
        for i in range(len(path)-1):
            dist += self.compute_distances(path[i], np.array(path[i+1]).reshape(-1, 2))
        return dist

    def generate_path(self, config1, config2):
        config1 = np.array(config1)[0:2]
        config2 = np.array(config2)[0:2]
        dist = np.linalg.norm(config2 - config1)
        if dist == 0:
            return config1.reshape(-1, 2), dist
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

    def shortcut(self, G, waypoints, num_trials=10):
        """
        Short cut waypoints if collision free
        @param waypoints list of node indices in the graph
        """
        # Implement here
        # 1. Choose two configurations
        # 2. Check for collision
        # 3. Connect them if collision free
        print("Originally {} waypoints".format(len(waypoints)))
        for _ in range(num_trials):
            if len(waypoints) == 2:
                break
            #for i in range(len(waypoints)):
            i = np.random.randint(0, len(waypoints) - 2)
            #for j in range(len(waypoints), i, -1):
            j = np.random.randint(i + 1, len(waypoints))
            start_node = waypoints[i]
            end_node = waypoints[j]

            valid, weight = self.edge_validity_checker(start_node, end_node)
            if valid:
                G.add_weighted_edges_from([(start_node, end_node, weight)]) # Add new edge to graph for future reference
                del waypoints[i+1:j] # Delete waypoints in between

        print("Path shortcut to {} waypoints".format(len(waypoints)))
        return waypoints


    def visualize_plan(self, G, path_nodes, start=None, goal=None, saveto="", plot_edges=False):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plan = []
        for node in path_nodes:
            #plan += [G.nodes[node]["config"]]
            plan += [node]
        plan = np.array(plan)
        #plt.clf()
        plt.imshow(self.map, interpolation='none', cmap='gray', origin='lower')

        # Comment this to hide all edges. This can take long.
        if plot_edges:
            edges = G.edges()
            for edge in edges:
                x = [config1[0], config2[0]]
                y = [config1[1], config2[1]]
                plt.plot(y, x, 'grey')

        path = self.get_path_on_graph(G, path_nodes)
        plt.plot(path[:,1], path[:,0], 'y', linewidth=1)

        for vertex in G.nodes:
            #config = G.nodes[vertex]["config"]
            if vertex == start:
                # Color the start node with blue
                plt.scatter(vertex[1], vertex[0], s=30, c='b')
            elif vertex == goal:
                # Color the goal node with green
                plt.scatter(vertex[1], vertex[0], s=30, c='g')
            else:
                plt.scatter(vertex[1], vertex[0], s=30, c='r')

        plt.tight_layout()

        if saveto != "":
            plt.savefig(saveto)
        else:
            plt.show()

    def visualize_graph(self, G, start=None, goal=None, added_node=None, ellipse=None, path_node=None, saveto="", plot_edges=False):
        plt.clf()
        ax = plt.gca()
        # hard coded
        bound = [[1767, 2223], [2312, 2793]]

        plt.imshow(self.map, interpolation='nearest', origin='lower')
        if plot_edges:
            edges = G.edges()
            for edge in edges:
                #config1 = G.nodes[edge[0]]["config"]
                #config2 = G.nodes[edge[1]]["config"]
                config1 = edge[0]
                config2 = edge[1]
                path = self.generate_path(config1, config2)[0]
                plt.plot(path[:,1], path[:,0], 'w')


        num_nodes = G.number_of_nodes()
        #ax = plt.gca()
        # print 'start', start
        # print 'goal', goal
        # print self.state_validity_checker(np.array(added_node))

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
            elif (added_node is not None) and (vertex in added_node):
                # Color the goal node with green
                plt.scatter(config[1], config[0], s=30, c='y')
            else:
                plt.scatter(config[1], config[0], s=30, c='r')
        if ellipse is not None:
            ellipse = [[ellipse[0][1], ellipse[0][0]], ellipse[1], ellipse[2], 90 - ellipse[3]]
            ell = Ellipse(*ellipse, color='pink', linewidth=3, alpha=0.5)
            ax.add_patch(ell)
        if path_node is not None:
            print path_node
            self.visualize_plan(G, path_node, start, goal, saveto=saveto)
        plt.xlim(bound[1])
        plt.ylim(bound[0])
        #plt.tight_layout()
        if saveto != "":
            plt.savefig(saveto)
        else:
            plt.ylabel('x')
            plt.xlabel('y')
            plt.show() #shape
