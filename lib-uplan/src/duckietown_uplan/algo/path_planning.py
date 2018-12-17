"""
This class is supposed to take care of everything for getting the initial path plan
"""
__all__ = [
    'PathPlanner',
]

import networkx as nx
import numpy as np
from shapely.geometry import Polygon
import geometry as geo
import networkx as nx
import copy


class PathPlanner(object):

    # def __init__(self, graph):
    #     self.graph = graph

    def __init__(self, graph, length=0.2, width=0.15,
                 node_to_index=None, index_to_node=None,
                 collision_matrix=None):
        self.graph = graph
        self.duckie_length = length
        self.duckie_width = width
        """
        TODO: the enumaration should come from the graph structure provided by Jose
        """
        self.node_to_index = node_to_index
        self.index_to_node = index_to_node
        self.collision_matrix = collision_matrix

    def _build_mod_graph(self, occupancy_vector):
        forbidden_vector = self.collision_matrix.dot(np.array(occupancy_vector))
        mod_graph = copy.deepcopy(self.graph)
        for node_idx in range(len(forbidden_vector)):
        # for node_name in nx.predecessor(mod_graph, ):
            if forbidden_vector[node_idx] != 0:
                for predecessor_node in mod_graph.predecessors(self.index_to_node[node_idx]):
                    mod_graph[predecessor_node][self.index_to_node[node_idx]][0]['dist'] = np.Inf
        return mod_graph

    def get_shortest_path(self, start, end, occupancy_node_names=[]):
        from duckietown_uplan.environment.utils import get_closest_neighbor
        # start_node_name, _ = get_closest_neighbor(self.graph, start)
        # path_node_names = nx.shortest_path(self.graph, start, end)
        # path_nodes = [(path_node_name, self.graph.nodes(data=True)[path_node_name])
        #               for path_node_name in path_node_names]
        # path_nodes = path_nodes[1:]
        # return path_nodes
        #
        #
        # print(occupancy_node_locs)

        #occupancy nodes should take care of the footprint of the duckie
        occupancy_vector = [0] * len(self.index_to_node)
        for occupied_node_name in occupancy_node_names:
            occupancy_vector[self.node_to_index[occupied_node_name]] = 1
        #create occupancy_vector
        mod_graph = self._build_mod_graph(occupancy_vector)
        path_node_names = nx.shortest_path(mod_graph, start, end, weight='dist')
        path_nodes = [(path_node_name, self.graph.nodes(data=True)[path_node_name])
                       for path_node_name in path_node_names]
        path_nodes = path_nodes[1:]
        return path_nodes

