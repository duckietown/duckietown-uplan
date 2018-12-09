
import copy
import contracts
import geometry as g
import numpy as np
import networkx as nx

import duckietown_world as dw
from duckietown_world.svg_drawing.ipython_utils import ipython_draw_html


def add_point_horizontally(point, dist):
    """ Takes a distance and a point and shifts the point to the
    left relative to his original position
    point: SE2Transform
    dist: float
    """
    shift_mat = g.SE2_from_rotation_translation(np.eye(2), np.array([0, dist]))
    point_mat = point.asmatrix2d().m
    shifted = np.dot(point_mat, shift_mat)
    return dw.geo.SE2Transform.from_SE2(shifted)


def augment_graph(graph, num_right=3, num_left=1, dist=0.1):

    aug_graph = copy.deepcopy(graph)

    # Add an augmented state for the graph
    for n in aug_graph.nodes():
        aug_graph .nodes[n]['visited'] = False

    first_node = list(graph.nodes())[0]
    to_visit_stack = [first_node, ]

    while True:
        if not to_visit_stack:
            break

        current_node = to_visit_stack.pop()

        # Add not visited successors
        for suc in nx.DiGraph.successors(graph, current_node):
            if not aug_graph.node[suc]['visited']:
                to_visit_stack.append(suc)

        if not aug_graph.node[current_node]['visited']:

            point_left = add_point_horizontally(aug_graph.node[current_node]['point'], -dist)
            point_right = add_point_horizontally(aug_graph.node[current_node]['point'], dist)

            current_node_l = current_node + "_l"
            current_node_r = current_node + "_r"

            aug_graph.add_node(current_node_l)
            aug_graph.add_node(current_node_r)
            aug_graph.node[current_node_l]['point'] = point_left
            aug_graph.node[current_node_r]['point'] = point_right
            predecessors = list(nx.DiGraph.predecessors(aug_graph, current_node))

            for predec in predecessors:
                if not aug_graph.node[predec]['visited']:
                    aug_graph.add_edge(predec, current_node_l)
                    aug_graph.add_edge(predec, current_node_r)
                else:
                    aug_graph.add_edge(predec, current_node_l)
                    aug_graph.add_edge(predec, current_node_r)
                    aug_graph.add_edge(predec + "_l", current_node_l)
                    aug_graph.add_edge(predec + "_r", current_node_r)
                    aug_graph.add_edge(predec + "_l", current_node)
                    aug_graph.add_edge(predec + "_r", current_node)

            aug_graph.node[current_node]['visited'] = True

    return aug_graph



