"""
This class is supposed to take care of everything for getting the velocity profiler plan
"""
import networkx as nx
import numpy as np
from duckietown_uplan.environment.footprint_table import FootprintTable
#from duckietown_uplan.algo.cost_function import


__all__ = [
    'VelocityProfiler',
]


class VelocityProfiler(object):
    def __init__(self, velocity_min, velocity_max, N):
        self.velocity_min = velocity_min
        self.velocity_max = velocity_max
        self.N = N
        self.vel_graph = None
        self.vel_ids = None
        self.vel_space = None
        self.path_ids = None

    def generate_velocity_graph(self, path):
        self.vel_graph = nx.DiGraph()
        self.vel_space = np.linspace(self.velocity_min, self.velocity_max, self.N)
        self.vel_ids = range(len(self.vel_space))
        self.path_ids = range(len(path))

        for node in self.path_ids:
            for vel_id in self.vel_ids:
                self.vel_graph.add_node((node, vel_id))

        for i, pose_id in enumerate(self.path_ids[:-1]):
            for j in self.vel_ids:
                from_node = (self.path_ids[i], self.vel_ids[j])
                for k in self.vel_ids:
                    to_node = (self.path_ids[i + 1], self.vel_ids[k])
                    self.vel_graph.add_edge(from_node, to_node, cost=1)

        return self.vel_graph

    def get_astar_path(self, vel_start, vel_end):
        # vel_0 and v_end are indices in the discretized velocity space (could )
        id_start = np.abs(self.vel_space - vel_start).argmin()
        id_end = np.abs(self.vel_space - vel_end).argmin()

        start = (self.path_ids[0], id_start)
        end = (self.path_ids[-1], id_end)

        return nx.astar_path(self.vel_graph, start, end, heuristic=None, weight='cost')

    def get_min_path(self, vel_start):
        min_path_cost = np.Inf
        min_path = []
        for end_vel in self.vel_ids:
            path = self.get_astar_path(vel_start, end_vel)
            path_cost = self.get_path_cost(path)
            if path_cost < min_path_cost:
                min_path_cost = path_cost
                min_path = path
        return min_path

    def get_path_cost(self, path):
        cost = 0
        for k, node in enumerate(path[-1]):
            cost += self.vel_graph[path[k]][path[k+1]]["cost"]
        return cost

    def get_velocity_profile(self, vel_start, path):
        self.vel_graph = self.generate_velocity_graph(path)
        min_path = self.get_min_path(vel_start)
        velocities = [self.vel_space[vel_id] for (pose_id, vel_id) in min_path]
        return velocities

