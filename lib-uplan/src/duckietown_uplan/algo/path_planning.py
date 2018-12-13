"""
This class is supposed to take care of everything for getting the initial path plan
"""
__all__ = [
    'PathPlanner',
]
import networkx as nx
from duckietown_uplan.environment.utils import get_closest_neighbor


class PathPlanner(object):
    def __init__(self, graph):
        self.graph = graph

    def build_collision_matrix(self):
        raise Exception("Path planner build_collision_matrix is not implemented yet")

    def get_shortest_path(self, start, end):
        start_nodes, _ = get_closest_neighbor(self.graph, start)
        start_node_name = start_nodes[0][0]
        path_node_names = nx.shortest_path(self.graph, start_node_name, end)
        path_nodes = [self.graph.nodes[path_node_name]['point'] for path_node_name in path_node_names]
        return path_nodes

    def block_nodes(self, ctrl_pts_to_block):
        raise Exception("Path planner get_shortest_path is not implemented yet")