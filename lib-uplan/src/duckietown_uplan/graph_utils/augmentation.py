
import copy
import geometry as g
import numpy as np
import networkx as nx

import duckietown_world as dw


class GraphAugmenter(object):
    # TODO: import constants

    width = 0.2

    @classmethod
    def add_point_horizontally(cls, point, dist):
        """ Takes a distance and a point and shifts the point to the
        left relative to his original position
        point: SE2Transform
        dist: float
        """
        shift_mat = g.SE2_from_rotation_translation(np.eye(2), np.array([0, dist]))
        point_mat = point.asmatrix2d().m
        shifted = np.dot(point_mat, shift_mat)
        return dw.geo.SE2Transform.from_SE2(shifted)

    @classmethod
    def add_attributes(cls):
        return False

    @classmethod
    def get_distance(cls, point, center_point):
        point_mat = point.asmatrix2d().m
        center_mat = center_point.asmatrix2d().m
        np.dot(point_mat, center_mat)

    @classmethod
    def is_in_lane(cls, dist2center):
        if abs(dist2center) >= cls.width/2:
            return False
        return True

    @classmethod
    def is_in_boundary(cls, dist2center):
        if dist2center >= -3*cls.width/2 and dist2center <= cls.width/2:
            return True
        return False

    @classmethod
    def to_dict(cls, graph):
        for node_name in graph.nodes():
            try:
                del graph.node[node_name]['visited']
            except KeyError:
                pass
        return {k: graph.node[node_name] for k, node_name in enumerate(graph.nodes())}

    @classmethod
    def augment_graph(cls, graph, num_right=1, num_left=1, dist=0.1):
        # Add a zero to the name of the original nodes
        mapping = {i: i+"_"+str(0) for i in graph.nodes()}
        graph = nx.relabel_nodes(graph, mapping)

        aug_graph = copy.deepcopy(graph)

        # Add an augmented state for the graph
        for n in aug_graph.nodes():
            # TODO: ADD FUNCTION THAT ASSIGNS THIS VALUES
            aug_graph.nodes[n]['center_node'] = n
            aug_graph.nodes[n]['p'] = aug_graph.nodes[n]['point'].p
            aug_graph.nodes[n]['theta'] = aug_graph.nodes[n]['point'].theta
            aug_graph.nodes[n]['index2center'] = 0
            aug_graph.nodes[n]['dist2center'] = 0
            aug_graph.nodes[n]['inlane'] = True
            aug_graph.nodes[n]['inboundary'] = True
            aug_graph.nodes[n]['visited'] = False

        first_node = list(graph.nodes())[0]
        to_visit_stack = [first_node, ]

        list_nodes_id = list(range(-num_left, num_right+1))

        while True:
            if not to_visit_stack:
                break

            current_node = to_visit_stack.pop()
            current_node_stem = current_node[:-2]  # Getting rig of  the _0

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
                        aug_graph.nodes[current_node_name]['center_node'] = current_node
                        aug_graph.nodes[current_node_name]['point'] = cls.add_point_horizontally(current_point, id*dist)
                        aug_graph.nodes[current_node_name]['p'] = current_point.p
                        aug_graph.nodes[current_node_name]['theta'] = current_point.theta
                        aug_graph.nodes[current_node_name]['index2center'] = id
                        aug_graph.nodes[current_node_name]['dist2center'] = id*dist
                        aug_graph.nodes[current_node_name]['inlane'] = cls.is_in_lane(id*dist)
                        aug_graph.nodes[current_node_name]['inboundary'] = cls.is_in_boundary(id*dist)

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

