import networkx as nx
import numpy as np
import pickle
import os
import time
from tqdm import tqdm
import multiprocessing as mp
from Dubins import dubins_path_planning
assert(nx.__version__ == '2.2' or nx.__version__ == '2.1' or nx.__version__ == '2.3')

def load_graph(filename):
    assert os.path.exists(filename)
    with open(filename, 'rb') as f:
        data = pickle.load(f)
        print('Loaded graph from {}'.format(f))
    return data['G']

def make_graph(env, sampler, connection_radius, num_vertices, lazy=False, saveto='graph.pkl', curvature=None):
    """
    Returns a graph on the passed environment.
    All vertices in the graph must be collision-free.

    Graph should have node attribute "config" which keeps a configuration in tuple.
    E.g., for adding vertex "0" with configuration np.array(0, 1),
    G.add_node(0, config=tuple(config))

    To add edges to the graph, call
    G.add_weighted_edges_from([edges])
    where edges is a list of tuples (node_i, node_j, weight),
    where weight is the distance between the two nodes.

    @param env: Map Environment for graph to be made on
    @param sampler: Sampler to sample configurations in the environment
    @param connection_radius: Maximum distance to connect vertices
    @param num_vertices: Minimum number of vertices in the graph.
    @param lazy: If true, edges are made without checking collision.
    @param saveto: File to save graph and the configurations
    @returns a undirected weighted graph G where each node is a tuple (x, y)
             and edge data the distance between nodes.
    """
    print('dubins graph maker')
    G = nx.DiGraph()
    # Implement here

    # TODO: Code needs to be restructured, maybe vectorized?
    # 1. Sample vertices
    vertices = sampler.sample(num_vertices)
    edges = []
    # 2. Connect them with edges
    start_time = time.time()
    for vid in tqdm(range(len(vertices))):
        vertex = tuple(vertices[vid])
        G.add_node(vertex)
        distances = env.compute_distances(vertices[vid], vertices)
        for vid2 in range(len(vertices)):
            if vid != vid2:
                dist = distances[vid2]
                if (dist < connection_radius) and (lazy or env.edge_validity_checker(vertices[vid], vertices[vid2])[0]):
                    edges.append((vertex, tuple(vertices[vid2]), dist))
    G.add_weighted_edges_from(edges)
    print ("Graph making time:", time.time() - start_time)
    # Check for connectivity.
    #num_connected_components = len(list(nx.connected_components(G)))
    #if not num_connected_components == 1:
    #    print ("warning, Graph has {} components, not connected".format(num_connected_components))

    # Save the graph to reuse.
    if saveto is not None:
        data = dict(G=G)
        pickle.dump(data, open(saveto, 'wb'))
        print('Saved the graph to {}'.format(saveto))
    return G

def add_node(G, config, env, connection_radius):
    """
    This function should add a node to an existing graph G.
    @param G graph, constructed using make_graph
    @param config Configuration to add to the graph
    @param env Environment on which the graph is constructed
    @param connection_radius Maximum distance to connect vertices
    """
    config = tuple(config)
    # new index of the configuration
    #index = G.number_of_nodes()
    #G.add_node(index, config=tuple(config))
    G.add_node(config)
    #G_configs = nx.get_node_attributes(G, 'config')
    #G_configs = [G_configs[node] for node in G_configs]

    # Implement here
    # Add edges from the newly added node
    edge_list = []
    for node in G.nodes():
        euc_dist = np.linalg.norm(np.array(node)[:2] - np.array(config)[:2])
        if euc_dist > connection_radius: continue
        edge_is_valid, dist = env.edge_validity_checker(config, node)
        if edge_is_valid and (1e-8 < dist < connection_radius):
            edge_list.append((config, node, dist))
        edge_is_valid, dist = env.edge_validity_checker(node, config)
        if edge_is_valid and (1e-8 < dist < connection_radius):
            edge_list.append((node, config, dist))
    G.add_weighted_edges_from(edge_list)
    # Check for connectivity.
    #num_connected_components = len(list(nx.connected_components(G)))
    #if not num_connected_components == 1:
    #    print ("warning, Graph has {} components, not connected".format(num_connected_components))
    return G, config

def add_node_lazy(G, config, env, connection_radius):
    """
    This function should add a node to an existing graph G.
    @param G graph, constructed using make_graph
    @param config Configuration to add to the graph
    @param env Environment on which the graph is constructed
    @param connection_radius Maximum distance to connect vertices
    """
    config = tuple(config)
    G.add_node(config)

    # Add edges from the newly added node
    edge_list = []
    for node in G.nodes():
        dist = env.compute_heuristic(config, node)
        if dist <= connection_radius:
            edge_list.append((config, node, dist))
        dist = env.compute_heuristic(node, config)
        if dist <= connection_radius:
            edge_list.append((node, config, dist))
    G.add_weighted_edges_from(edge_list)
    return G, config
