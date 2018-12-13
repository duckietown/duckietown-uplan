"""
This class is supposed to take care of everything for getting the initial path plan
"""
__all__ = [
    'PathPlanner',
]

import numpy as np
from shapely.geometry import Polygon
import geometry as geo
import networkx as nx
import copy

class PathPlanner(object):
    def __init__(self, graph, occupancy_vector, length=0.2, width=0.15):
        self.graph = graph
        self.duckie_length = length
        self.duckie_width = width
        self.occupancy_vector = occupancy_vector
        # raise Exception("Path planner init is not implemented yet")

    def build_collision_matrix(self):

        def local_to_global(poly_points, q):
            global_points = []
            for points in poly_points:
                q_point = geo.SE2_from_translation_angle(points, 0.0)
                global_point, _ = geo.translation_angle_from_SE2(geo.SE2.multiply(q, q_point))
                global_points.append(global_point)

            return global_points

        def overlap(q1, q2):
            poly_points = ([((self.duckie_length / 2.0), (-self.duckie_width / 2.0)),
                            ((self.duckie_length / 2.0), (self.duckie_width / 2.0)),
                            ((-self.duckie_length / 2.0), (self.duckie_width / 2.0)),
                            ((-self.duckie_length / 2.0), (-self.duckie_width / 2.0))])

            poly1 = Polygon(local_to_global(poly_points, q1))
            poly2 = Polygon(local_to_global(poly_points, q2))

            return 1 if poly1.intersects(poly2) else 0

        num_nodes = len(self.graph)
        collision_matrix = [[0 for ni in range(num_nodes)] for nj in range(num_nodes)]

        for ni, i in enumerate(self.graph):
            curr = self.graph.nodes[i]['point'].as_SE2()
            for nj, j in enumerate(self.graph):
                if i < j:
                    break
                elif i == j:
                    collision_matrix[ni][nj] = 1
                else:
                    q = self.graph.nodes[j]['point'].as_SE2()
                    collision = overlap(curr, q)
                    collision_matrix[ni][nj] = collision
                    collision_matrix[nj][ni] = collision

        return collision_matrix

        # raise Exception("Path planner build_collision_matrix is not implemented yet")

    def build_mod_graph(self):
        collision_matrix = np.array(self.build_collision_matrix())
        forbidden_vector = collision_matrix.dot(np.array(self.occupancy_vector))
        mod_graph = copy.deepcopy(self.graph)
        for ni,node in  enumerate(mod_graph):

            if forbidden_vector[ni]!=0:
                for node2 in mod_graph.predecessors(node)
                    mod_graph[node2][node][0]['blocked_weight'] = np.infty
            else:
                for node2 in mod_graph.predecessors(node)
                    mod_graph[node2][node][0]['blocked_weight'] = 1 # or actual distance if we want

        return mod_graph


    def get_shortest_path(self, start, end):
        mod_graph = self.build_mod_graph()
        path = nx.shortest_path(mod_graph, start, end, weight = 'blocked_weight') #shortest collision free path

        return path
        # raise Exception("Path planner get_shortest_path is not implemented yet")