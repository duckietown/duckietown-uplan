"""
This class is supposed to take care of everything for getting the velocity profiler plan
"""
import networkx as nx
import numpy as np
from duckietown_uplan.environment.footprint_table import FootprintTable


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
        self.path_history = None
        self.trajectory_history = None
        self.uncertainties = []
        self.cost_array = []
        self.additive_cost_array = []

    def generate_velocity_graph(self, path, cost_function=None):

        if cost_function is None:
            raise ValueError("cost function needed")

        self.vel_graph = nx.DiGraph()
        self.vel_space = np.linspace(self.velocity_min, self.velocity_max, self.N)
        self.vel_ids = range(len(self.vel_space))
        self.path_ids = range(len(path))
        uncertainties = self.get_toy_uncertainties(len(path))  # TODO erase this toy example
        print("delta_v_norm" + str(uncertainties))

        for node in self.path_ids:
            for vel_id in self.vel_ids:
                self.vel_graph.add_node((node, vel_id))

        for i, pose_id in enumerate(self.path_ids[:-1]):
            for j in self.vel_ids:
                from_node = (self.path_ids[i], self.vel_ids[j])
                for k in self.vel_ids:
                    to_node = (self.path_ids[i + 1], self.vel_ids[k])
                    start_vel = self.vel_space[self.vel_ids[j]]
                    next_vel = self.vel_space[self.vel_ids[k]]

                    # TODO: check logic of
                    delta_v = np.abs(start_vel - next_vel)
                    delta_v_norm = delta_v/(self.velocity_max - self.velocity_min)
                    delta_unc = np.abs(uncertainties[i] - uncertainties[i+1])
                    ref = self.velocity_max
                    error = abs(ref - next_vel)
                    error_norm = error/(self.velocity_max - self.velocity_min)

                    # Penalizing huge changes in velocity
                    #if abs(self.vel_ids[j] - self.vel_ids[k]) > 1:
                    #    delta_v_norm = np.Inf

                    a1 = 8
                    a2 = 0
                    a3 = 8
                    a4 = 0
                    a5 = 10

                    next_vel_norm = (next_vel-self.velocity_min)/(self.velocity_max - self.velocity_min)

                    # cost = self.get_cost(delta_v_norm=a1*delta_v_norm**3,
                    #                      delta_unc=delta_unc,
                    #                      unc=a3*next_vel_norm*uncertainties[i]**2,
                    #                      vel=a4*next_vel_norm**2,
                    #                      error=a5*error_norm**2)

                    cost = cost_function(delta_v_norm=delta_v_norm,
                                         delta_unc=delta_unc,
                                         unc=next_vel_norm*uncertainties[i],
                                         next_vel_norm=next_vel_norm,
                                         error_norm=error_norm)

                    self.vel_graph.add_edge(from_node, to_node,
                                            cost=cost,
                                            delta_v_norm=a1*delta_v_norm**2,
                                            delta_unc=delta_unc,
                                            unc=a3*uncertainties[i]**2,
                                            vel=a4*((next_vel-self.velocity_min)/(self.velocity_max - self.velocity_min))**2,
                                            error=a5*error_norm**2)

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
        path_history = []
        self.cost_array = []
        self.additive_cost_array = []
        for end_vel_id in self.vel_ids:
            path = self.get_astar_path(vel_start, self.vel_space[end_vel_id])
            path_cost, cost_array, additive_cost_array = self.get_path_cost(path)
            path_history.append((path_cost, path))

            if path_cost < min_path_cost:
                min_path_cost = path_cost
                min_path = path
                self.cost_array = cost_array
                self.additive_cost_array = additive_cost_array

        if min_path == np.Inf:
            raise ValueError("No feasable velocity path")
        return min_path, min_path_cost, path_history

    def get_path_cost(self, path):
        additive_cost = 0
        cost_array = []
        additive_cost_array = []
        for k, node in enumerate(path[:-1]):
            additive_cost_array.append(additive_cost)
            cost_array.append(self.vel_graph[path[k]][path[k+1]]["cost"])
            additive_cost += cost_array[-1]

        return additive_cost, cost_array, additive_cost_array

    def get_velocity_profile(self, vel_start, path, cost_function=lambda x: 0):
        self.vel_graph = self.generate_velocity_graph(path, cost_function=cost_function)
        min_path, min_path_cost, self.path_history = self.get_min_path(vel_start)

        min_trajectory = self.get_trajectory_from_path(min_path)

        self.trajectory_history = []
        for cost, p in self.path_history:
            self.trajectory_history.append((cost, self.get_trajectory_from_path(p)))

        return min_trajectory

    def get_trajectory_from_path(self, path):
        return [self.vel_space[vel_id] for (pose_id, vel_id) in path]


    @staticmethod
    def get_cost(delta_v_norm=0, delta_unc=0.0, vel=0.0, error=0, unc=0):
        return delta_v_norm + unc + vel + error

    def get_toy_uncertainties(self, num=10):

        f = lambda x: 0.5 - 0.5*np.cos(x)
        t = np.linspace(0, 4*np.pi, num)

        #f = lambda x: x**2
        #t = np.linspace(0,1,num)
        #y = [f(x) for x in t]

        #first_list = np.linspace(0,1,np.ceil(num/2))
        #rest = np.flip(first_list)
        #final = np.concatenate((first_list, rest), axis=None)

        #return np.random.rand(num)
        self.uncertainties = [f(x) for x in t]
        return self.uncertainties



