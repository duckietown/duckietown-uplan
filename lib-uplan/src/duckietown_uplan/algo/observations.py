"""
Each duckiebot will have its own observation model to update it
"""
__all__ = [
    'ObservationModel',
]

import collections
observation_memory_length = 200
initial_obstacle_prob = 0.2
discount_factor = 0.00005


class ObservationModel(object):
    def __init__(self, map):
        self.graph = map
        self.curr_uncertainty_values = dict()
        for key in self.graph.nodes:
            self.curr_uncertainty_values[key] = [self.graph.nodes(data=True)[key], 0.2]
        self.observations_history = collections.deque(maxlen=observation_memory_length)

    def reset_obstacles_uncertainity(self):
        for key in self.curr_uncertainty_values.keys():
            self.curr_uncertainty_values[key] = [self.graph.nodes(data=True)[key], 0.2]

    def get_uncertainty_from_node(self, node_name):
        return self.curr_uncertainty_values[node_name][1]

    def update_obstacles_uncertainity(self, current_observations):
        self.update_observations_history(current_observations.keys())
        for key, value in current_observations.items(): ## Returns a dict()
            self.curr_uncertainty_values[key][1] = value
            # now we need to update the uncertainities for the last t steps from the observed_nodes_history if not observed
        for timestep in self.observations_history:
            for node_name in timestep:
                # get its name
                if node_name in current_observations:
                    continue
                # reduce it if its higher than the initial obstacle prob and increase otherwise
                if self.curr_uncertainty_values[node_name][1] > initial_obstacle_prob:
                    self.curr_uncertainty_values[node_name][1] = self.curr_uncertainty_values[node_name][1] * (1 - discount_factor)
                elif self.curr_uncertainty_values[node_name][1] < initial_obstacle_prob:
                    self.curr_uncertainty_values[node_name][1] = self.curr_uncertainty_values[node_name][1] + (
                                initial_obstacle_prob / observation_memory_length)
                else:
                    continue
        return

    def update_observations_history(self, current_observed_node_names):
        self.observations_history.append(current_observed_node_names)

    def forget_observations_history(self):
        self.observations_history = collections.deque(maxlen=observation_memory_length)

    def get_path_uncertainities(self, path):
        path_uncertainities = [self.curr_uncertainty_values[path_node[0]][1] for path_node in path]
        return path_uncertainities

    def get_map_uncertainities(self, path):
        return self.curr_uncertainty_values
