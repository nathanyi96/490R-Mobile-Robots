#!/usr/bin/env python

import argparse, numpy, time
import networkx as nx
import math
import numpy as np
import dubins_graph_maker as graph_maker
import astar
import lazy_astar
from DubinsSampler import DubinsSampler
from DubinsMapEnvironment import DubinsMapEnvironment



# This is for running DubinsPlanner without ROS.
# python runDubins.py -m ../maps/map1.txt -s 0 0 0 -g 8 7 90 --num-vertices 15
# python runDubins.py -m ../maps/map2.txt -s 321 148 0 -g 106 202 90 --num-vertices 500 --connection-radius 100

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='map1.txt',
                        help='The environment to plan on')
    parser.add_argument('-s', '--start', nargs='+', type=int, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=int, required=True)
    parser.add_argument('-c', '--curvature', type=float, default=3)
    parser.add_argument('--num-vertices', type=int, required=True)
    parser.add_argument('--connection-radius', type=float, default=15.0)
    parser.add_argument('--lazy', action='store_true')

    args = parser.parse_args()
    args.start[2] = math.radians(args.start[2])
    args.goal[2] = math.radians(args.goal[2])


    map_data = np.loadtxt(args.map).astype(np.int)

    # First setup the environment and the robot.
    planning_env = DubinsMapEnvironment(map_data, args.curvature)

    G = graph_maker.make_graph(planning_env,
        sampler=DubinsSampler(planning_env),
        num_vertices=args.num_vertices,
        connection_radius=args.connection_radius,
        lazy=args.lazy)

    G, start_id = graph_maker.add_node(G, args.start, env=planning_env,
        connection_radius=args.connection_radius)
    G, goal_id = graph_maker.add_node(G, args.goal, env=planning_env,
        connection_radius=args.connection_radius)

    # Uncomment this to visualize the graph
    planning_env.visualize_graph(G,start_id, goal_id)

    try:
        heuristic = lambda n1, n2: planning_env.compute_heuristic(
            #G.nodes[n1]['config'], G.nodes[n2]['config'])
            n1, n2)
        start_time = time.time()
        if args.lazy:
            weight = lambda n1, n2: planning_env.edge_validity_checker(
                #G.nodes[n1]['config'], G.nodes[n2]['config'])
                n1, n2)
            path, dist = lazy_astar.astar_path(G,
                source=start_id, target=goal_id, weight=weight, heuristic=heuristic)
        else:
            path, dist = astar.astar_path(G,
                source=start_id, target=goal_id, heuristic=heuristic)
        print('plan time: ', time.time() - start_time)
        print('path length: {}'.format(dist))
        planning_env.visualize_plan(G, path, start_id, goal_id)
    except nx.NetworkXNoPath as e:
        print(e)
