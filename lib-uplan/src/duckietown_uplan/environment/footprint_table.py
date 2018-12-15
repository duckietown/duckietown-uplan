
__all__ = [
    'FootprintTable',
]

import numpy as np


class FootprintTable(object):

    def __init__(self, graph, radius):
        self.graph = graph
        self.radius = radius
        self.data = {}
        self.create_dictionary()

    def in_radius(self, center_node):
        nodes_in_radius = []
        center_point = self.graph.node[center_node]["point"]
        for node in self.graph.nodes():
            point= self.graph.node[node]["point"]
            if self.get_euclidean_distance(center_point, point) < self.radius:
                nodes_in_radius.append(node)
        return nodes_in_radius

    def create_dictionary(self):
        for node in self.graph.nodes():
            self.data[node] = self.in_radius(node)

    def get_data(self):
        return self.data

    @staticmethod
    def get_euclidean_distance(point1, point2):
        return np.linalg.norm(point1.p - point2.p)
