#!/usr/bin/env python

import argparse, numpy, time
import networkx as nx
import numpy as np

from MapEnvironment import MapEnvironment
import graph_maker
import astar
import lazy_astar
from Sampler import Sampler


# Try running the following
# python run.py -m ../maps/map1.txt -s 0 0 -g 8 7 --num-vertices 15
# python run.py -m ../maps/map2.txt -s 321 148 -g 106 202 --num-vertices 250 --connection-radius 100

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='../maps/map1.txt',
                        help='The environment to plan on')
    parser.add_argument('-s', '--start', nargs='+', type=int, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=int, required=True)
    parser.add_argument('--num-vertices', type=int, required=True)
    parser.add_argument('--connection-radius', type=float, default=20.0)
    parser.add_argument('--lazy', action='store_true')

    args = parser.parse_args()

    # First setup the environment
    map_data = np.loadtxt(args.map).astype(np.int)
    planning_env = MapEnvironment(map_data)

    # Make a graph
    G = graph_maker.make_graph(planning_env,
        sampler=Sampler(planning_env),
        num_vertices=args.num_vertices,
        connection_radius=args.connection_radius,
        lazy=args.lazy)

    # Add start and goal nodes
    print(args.start, args.goal)
    G, start_id = graph_maker.add_node(G, args.start, env=planning_env,
        connection_radius=args.connection_radius)
    G, goal_id = graph_maker.add_node(G, args.goal, env=planning_env,
        connection_radius=args.connection_radius)

    # Uncomment this to visualize the graph
    #planning_env.visualize_graph(G, tuple(args.start), tuple(args.goal))

    try:
        heuristic = planning_env.compute_heuristic
        #heuristic = lambda n1, n2: planning_env.compute_heuristic(
        #    G.nodes[n1]['config'], G.nodes[n2]['config'])
        #    n1, n2)
        end = time.time()
        if args.lazy:
            print("lazy A*")
            weight = planning_env.edge_validity_checker
            #weight = lambda n1, n2: planning_env.edge_validity_checker(n1, n2)
            #    G.nodes[n1]['config'], G.nodes[n2]['config'])
            path = lazy_astar.astar_path(G,
                source=start_id, target=goal_id, weight=weight, heuristic=heuristic)
        else:
            path = astar.astar_path(G,
                source=start_id, target=goal_id, heuristic=heuristic)
        print('plan time: ', time.time() - end)
        #print("path we got", path)
        planning_env.visualize_plan(G, path, start_id, goal_id)
    except nx.NetworkXNoPath as e:
        print(e)
