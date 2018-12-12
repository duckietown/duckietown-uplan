
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


def augment_graph(graph, num_right=1, num_left=1, dist=0.1):
    # Add a zero to the name of the original nodes
    mapping = {i: i+"_"+str(0) for i in graph.nodes()}
    graph = nx.relabel_nodes(graph, mapping)

    aug_graph = copy.deepcopy(graph)


    # Add an augmented state for the graph
    for n in aug_graph.nodes():
        aug_graph.nodes[n]['visited'] = False

    first_node = list(graph.nodes())[0]
    to_visit_stack = [first_node, ]

    list_nodes_id = list(range(-num_left, num_right+1))

    while True:
        if not to_visit_stack:
            break

        current_node = to_visit_stack.pop()
        current_node_stem = current_node[:-2]

        # Add not visited successors
        for suc in nx.DiGraph.successors(graph, current_node):
            if not aug_graph.node[suc]['visited']:
                to_visit_stack.append(suc)

        if not aug_graph.node[current_node]['visited']:
            current_point = aug_graph.node[current_node]['point']

            for id in list_nodes_id:
                if id != 0:    # Not the center node
                    current_node_name = current_node_stem + "_" + str(id)
                    aug_graph.add_node(current_node_name)
                    aug_graph.node[current_node_name]['point'] = add_point_horizontally(current_point, id*dist)

            for pre in nx.DiGraph.predecessors(graph, current_node):
                pre_stem = pre[:-2]
                if -1 in list_nodes_id:
                    aug_graph.add_edge(pre, current_node_stem + "_-1")
                if 1 in list_nodes_id:
                    aug_graph.add_edge(pre, current_node_stem + "_1")

                if aug_graph.node[pre]['visited']:

                    # Connect with straight edges
                    for id in list_nodes_id:
                        aug_graph.add_edge(pre_stem + "_" + str(id), current_node_stem + "_" + str(id))

                    # Connect diagonally to the right
                    for id in list_nodes_id[1:]:
                        aug_graph.add_edge(pre_stem + "_" + str(id-1), current_node_stem + "_" + str(id))

                    # Connect diagonally to the left
                    for id in list_nodes_id[:-1]:
                        aug_graph.add_edge(pre_stem + "_" + str(id+1), current_node_stem + "_" + str(id))

            # Check if succesors are visited and add nodes from new neighbors to their neighbors
            for suc in nx.DiGraph.successors(graph, current_node):
                suc_stem = suc[:-2]
                if aug_graph.node[suc]['visited']:
                    # Connect with straight edges
                    for id in list_nodes_id:
                        aug_graph.add_edge(current_node_stem + "_" + str(id), suc_stem + "_" + str(id))

                    # Connect diagonally to the right
                    for id in list_nodes_id[1:]:
                        aug_graph.add_edge(current_node_stem + "_" + str(id-1), suc_stem + "_" + str(id))

                    # Connect diagonally to the left
                    for id in list_nodes_id[:-1]:
                        aug_graph.add_edge(current_node_stem + "_" + str(id+1), suc_stem + "_" + str(id))

        aug_graph.node[current_node]['visited'] = True

    return aug_graph



