"""
This class is supposed to take care of everything for getting the velocity profiler plan
"""
import networkx as nx
import numpy as np
from duckietown_uplan.environment.footprint_table import FootprintTable
# from duckietown_uplan.algo.cost_function import cost_function


__all__ = [
    'VelocityProfiler',
]


class VelocityProfiler(object):
    def __init__(self, graph, path):
        raise Exception("Velocity profiler init is not implemented yet")
        # self.G = graph
        # self.path = path

    def get_velocity_profile(self, path, velocity_min, velocity_max, velocity_discretization_space):
        raise Exception("Velocity profiler get_velocity_profile is not implemented yet")
        # vel_G = nx.DiGraph()
        #
        # vel_space = np.linspace(velocity_min, velocity_max, velocity_discretization_space)
        #
        # #     lanes = get_lanes(path);
        #
        # vel_G.add_node((1, 1));
        #
        # for i in range(len(path) - 1):
        #     for j, vel in enumerate(vel_space):
        #         vel_G.add_node((i + 2, j + 1))
        #
        # for (i, j) in vel_G:
        #     if i == 1:
        #         for k, vel in enumerate(vel_space):
        #             vel_G.add_edge((1, 1), (2, k + 1), weight=cost_function())
        #     elif i < len(path):
        #         for k, vel in enumerate(vel_space):
        #             vel_G.add_edge((i, j), (i + 1, k + 1), weight=cost_function())
        # return vel_G

    def get_edge_velocity(self, vel_0, vel_end):
        raise Exception("Velocity profiler get_velocity_profile is not implemented yet")
            # vel_0 and v_end are indices in the discretized velocity space (could )
            # nx.astar_path(vel_G, (1, vel_0), (len(path), vel_end), heuristic=None, weight='weight')

    #TODO: convert index in path to node so that you can send to cost function
    #TODO: cost_function (increase radius in footprint table the farther the node is)
