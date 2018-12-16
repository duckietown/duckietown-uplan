
import copy
import geometry as geo
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
        shift_mat = geo.SE2_from_rotation_translation(np.eye(2), np.array([0, dist]))
        point_mat = point.asmatrix2d().m
        shifted = np.dot(point_mat, shift_mat)
        return dw.geo.SE2Transform.from_SE2(shifted)

    @classmethod
    def add_attributes(cls):
        return False

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
    def interpolate(cls, q0, q1, alpha):
        v = geo.SE2.algebra_from_group(geo.SE2.multiply(geo.SE2.inverse(q0), q1))
        vi = v * alpha
        q = np.dot(q0, geo.SE2.group_from_algebra(vi))
        return q

    @classmethod
    def interpolate_segment(cls, start_pose, end_pose, n=2):
        """
        Returns list of n poses following a path from start_pose to end_pose
        :param start_pose:
        :param end_pose:
        :param n: number of poses between start pose and end pose
        :return: list of poses in order
        """
        # create a sequence of poses
        start_pose_p = start_pose.as_SE2()
        end_pose_p = end_pose.as_SE2()
        seqs = []
        steps = np.linspace(0, 1, num=n)
        for alpha in steps:
            q = cls.interpolate(start_pose_p, end_pose_p, alpha)
            seqs.append(q)
        return seqs

    @classmethod
    def augment_graph(cls, graph, num_long=1, num_right=1, num_left=1, lat_dist=0.1):
        assert num_long >= 0
        assert num_left >= 0
        assert num_right >= 0
        # Renaming nodes
        long_graph = cls.augment_graph_longitudinal(graph, num_long)
        aug_graph = cls.augment_graph_lateral(long_graph, num_right, num_left, lat_dist)

        # Set edge weight
        for start, end, k in aug_graph.edges(keys=True):
            start_p = aug_graph.node[start]['point'].p
            end_p = aug_graph.node[end]['point'].p
            aug_graph[start][end][k]['dist'] = np.linalg.norm(start_p - end_p)

        return aug_graph

    @classmethod
    def augment_graph_longitudinal(cls, orig_graph, num_long=0):
        """
        Augment graph longitudinal, this means adding nodes between original nodes
        :param graph:
        :param num_long: number of nodes in between original nodes
        :return: logitudinally augmented graph
        """
        graph = copy.deepcopy(orig_graph)
        mapping = {node: (k, 0) for k, node in enumerate(graph.nodes())}
        graph = nx.relabel_nodes(graph, mapping)

        num_long = num_long + 2
        for node in list(graph.nodes()):
            edges_out = graph.out_edges(node)
            for start, end in list(edges_out):
                start_pose = graph.node[start]['point']
                end_pose = graph.node[end]['point']
                sequence_poses = cls.interpolate_segment(start_pose, end_pose, n=num_long)
                new_node_ids = [(start[0], end[0], k) for k in range(len(sequence_poses))]

                # Get rid of start and end node
                new_node_ids = new_node_ids[1:-1]
                sequence_poses = sequence_poses[1:-1]
                if len(new_node_ids) == 0:
                    continue

                for k, new_node_id in enumerate(new_node_ids):
                    if k == 0:
                        graph.add_node(new_node_ids[k], point=dw.SE2Transform.from_SE2(sequence_poses[k]))
                    else:
                        graph.add_node(new_node_ids[k], point=dw.SE2Transform.from_SE2(sequence_poses[k]))
                        graph.add_edge(new_node_ids[k-1], new_node_ids[k])
                graph.add_edge(start, new_node_ids[0])
                graph.add_edge(new_node_ids[-1], end)
                graph.remove_edge(start, end)

        return graph

    @classmethod
    def augment_graph_lateral(cls, graph, num_right=1, num_left=1, lat_dist=0.1):

        mapping = {node: (k, 0) for k, node in enumerate(graph.nodes())}
        graph = nx.relabel_nodes(graph, mapping)
        aug_graph = copy.deepcopy(graph)

        # Add an augmented state for the graph
        for n in aug_graph.nodes():
            # TODO: ADD FUNCTION THAT ASSIGNS THIS VALUES
            aug_graph.nodes[n]['center_node'] = n[0]
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

            center_node = to_visit_stack.pop()

            # Add not visited successors
            for suc in nx.DiGraph.successors(graph, center_node):
                if not aug_graph.node[suc]['visited']:
                    to_visit_stack.append(suc)

            if not aug_graph.node[center_node]['visited']:
                current_point = aug_graph.node[center_node]['point']

                for id in list_nodes_id:
                    if id != 0:    # Not the center node
                        aug_graph.add_node((center_node[0], id))
                        aug_graph.nodes[(center_node[0], id)]['center_node'] = center_node[0]
                        aug_graph.nodes[(center_node[0], id)]['point'] = cls.add_point_horizontally(current_point, id * lat_dist)
                        aug_graph.nodes[(center_node[0], id)]['p'] = current_point.p
                        aug_graph.nodes[(center_node[0], id)]['theta'] = current_point.theta
                        aug_graph.nodes[(center_node[0], id)]['index2center'] = id
                        aug_graph.nodes[(center_node[0], id)]['dist2center'] = id * lat_dist
                        aug_graph.nodes[(center_node[0], id)]['inlane'] = cls.is_in_lane(id * lat_dist)
                        aug_graph.nodes[(center_node[0], id)]['inboundary'] = cls.is_in_boundary(id * lat_dist)

                for pre in nx.DiGraph.predecessors(graph, center_node):
                    if -1 in list_nodes_id:
                        aug_graph.add_edge(pre, (center_node[0], -1))
                    if 1 in list_nodes_id:
                        aug_graph.add_edge(pre, (center_node[0], 1))

                    if aug_graph.node[pre]['visited']:
                        # Connect with straight edges
                        for id in list_nodes_id:
                            aug_graph.add_edge((pre[0], id), (center_node[0], id))

                        # Connect diagonally to the right
                        for id in list_nodes_id[1:]:
                            aug_graph.add_edge((pre[0], id-1), (center_node[0], id))

                        # Connect diagonally to the left
                        for id in list_nodes_id[:-1]:
                            aug_graph.add_edge((pre[0], id+1), (center_node[0], id))

                # Check if succesors are visited and add nodes from new neighbors to their neighbors
                for suc in nx.DiGraph.successors(graph, center_node):
                    if aug_graph.node[suc]['visited']:
                        # Connect with straight edges
                        for id in list_nodes_id:
                            aug_graph.add_edge((center_node[0], id), (suc[0], id))

                        # Connect diagonally to the right
                        for id in list_nodes_id[1:]:
                            aug_graph.add_edge((center_node[0], id-1), (suc[0], id))

                        # Connect diagonally to the left
                        for id in list_nodes_id[:-1]:
                            aug_graph.add_edge((center_node[0], id+1), (suc[0], id))

            aug_graph.node[center_node]['visited'] = True

        return aug_graph

