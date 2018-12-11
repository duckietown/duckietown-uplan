
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


def vanilla_augment_graph(graph, dist=0.1):
    aug_graph = copy.deepcopy(graph)

    # Add an augmented state for the graph
    for n in aug_graph.nodes():
        aug_graph.nodes[n]['visited'] = False

    first_node = list(graph.nodes())[0]
    to_visit_stack = [first_node, ]

    while True:
        if not to_visit_stack:
            break

        current_node = to_visit_stack.pop()

        # Add not visited successors to stack
        for suc in nx.DiGraph.successors(graph, current_node):
            if not aug_graph.node[suc]['visited']:
                to_visit_stack.append(suc)

        if not aug_graph.node[current_node]['visited']:

            current_point = aug_graph.node[current_node]['point']

            aug_graph.add_node(current_node + "_l")
            aug_graph.add_node(current_node + "_r")
            aug_graph.node[current_node + "_l"]['point'] = add_point_horizontally(current_point, -dist)
            aug_graph.node[current_node + "_r"]['point'] = add_point_horizontally(current_point, dist)

            for predec in nx.DiGraph.predecessors(graph, current_node):
                # Add edges from parent to new neighbors
                aug_graph.add_edge(predec, current_node + "_l")
                aug_graph.add_edge(predec, current_node + "_r")
                if aug_graph.node[predec]['visited']:
                    aug_graph.add_edge(predec + "_r", current_node + "_r")
                    aug_graph.add_edge(predec + "_l", current_node + "_l")
                    aug_graph.add_edge(predec + "_r", current_node)
                    aug_graph.add_edge(predec + "_l", current_node)

            # Check if succesors are visited and add nodes from new neighbors to their neighbors
            for suc in nx.DiGraph.successors(graph, current_node):
                if aug_graph.node[suc]['visited']:
                    aug_graph.add_edge(current_node + "_r", suc + "_r")
                    aug_graph.add_edge(current_node + "_l", suc + "_l")
                    aug_graph.add_edge(current_node + "_r", suc)
                    aug_graph.add_edge(current_node + "_l", suc)

        aug_graph.node[current_node]['visited'] = True

    return aug_graph

#
# def augment_graph(graph, num_right=3, num_left=1, dist=0.1):
#
#     aug_graph = copy.deepcopy(graph)
#
#     # Add an augmented state for the graph
#     for n in aug_graph.nodes():
#         aug_graph .nodes[n]['visited'] = False
#
#     first_node = list(graph.nodes())[0]
#     to_visit_stack = [first_node, ]
#
#     list_right_points = []
#     list_right_names = []
#     list_left_points = []
#     list_left_names = []
#
#     while True:
#         if not to_visit_stack:
#             break
#
#         current_node = to_visit_stack.pop()
#
#         # Add not visited successors
#         for suc in nx.DiGraph.successors(graph, current_node):
#             if not aug_graph.node[suc]['visited']:
#                 to_visit_stack.append(suc)
#
#         if not aug_graph.node[current_node]['visited']:
#
#             current_point = aug_graph.node[current_node]['point']
#             for d in range(num_left):
#                 current_node_name = current_node + "_l" + str(d)
#                 list_left_points.append(add_point_horizontally(current_point, -d*dist))
#                 list_left_names.append(current_node_name)
#                 aug_graph.add_node(current_node_name)
#                 aug_graph.node[current_node_name]['point'] = list_left_points[-1]
#
#             for d in range(num_right):
#                 current_node_name = current_node + "_r" + str(d)
#                 list_right_points.append(add_point_horizontally(current_point, d*dist))
#                 list_right_names.append(current_node_name)
#                 aug_graph.add_node(current_node_name)
#                 aug_graph.node[current_node_name]['point'] = list_right_points[-1]
#
#             predecessors = list(nx.DiGraph.predecessors(aug_graph, current_node))
#
#             for predec in predecessors:
#                 if not aug_graph.node[predec]['visited']:
#                     if num_left != 0:
#                         aug_graph.add_edge(predec, current_node + "_l0")
#                     if num_right != 0:
#                         aug_graph.add_edge(predec, current_node + "_r0")
#                 else:
#                     if num_left != 0:
#                         aug_graph.add_edge(predec, current_node + "_l0")
#                     if num_right != 0:
#                         aug_graph.add_edge(predec, current_node + "_r0")
#                     for i in range(num_left):
#                         aug_graph.add_edge(predec + "_l" + str(i), current_node + "_l" + str(i))
#                     for i in range(num_right):
#                         aug_graph.add_edge(predec + "_r" + str(i), current_node + "_r" + str(i))
#
#                     #aug_graph.add_edge(predec + "_r", current_node_r)
#                     #aug_graph.add_edge(predec + "_l", current_node)
#                     #aug_graph.add_edge(predec + "_r", current_node)
#
#             aug_graph.node[current_node]['visited'] = True
#
#     return aug_graph



