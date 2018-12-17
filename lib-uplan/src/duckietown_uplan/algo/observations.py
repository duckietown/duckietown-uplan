"""
Each duckiebot will have its own observation model to update it
"""
__all__ = [
    'ObservationModel',
]

import collections
observation_memory_length = 15
initial_obstacle_prob = 0.2
discount_factor = 0.8


class ObservationModel(object):
    def __init__(self, map):
        self.graph = map
        self.curr_uncertainty_values = dict()
        for key in self.graph.nodes:
            self.curr_uncertainty_values[key] = 0.2
        self.observations_history = collections.deque(maxlen=observation_memory_length)

    def reset_obstacles_uncertainity(self):
        for key in self.curr_uncertainty_values.keys():
            self.curr_uncertainty_values[key] = initial_obstacle_prob

    def get_uncertainty_from_node(self, node_name):
        return self.curr_uncertainty_values[node_name]

    def update_obstacles_uncertainity(self, current_observations):
        self.update_observations_history(current_observations.keys())
        for key, value in current_observations.items(): ## Returns a dict()
            self.curr_uncertainty_values[key] = value
            # now we need to update the uncertainities for the last t steps from the observed_nodes_history if not observed
        for timestep in self.observations_history:
            for node_name in timestep:
                # get its name
                if node_name in current_observations:
                    continue
                # reduce it if its higher than the initial obstacle prob and increase otherwise
                if self.curr_uncertainty_values[node_name] > initial_obstacle_prob:
                    self.curr_uncertainty_values[node_name] = self.curr_uncertainty_values[node_name] * discount_factor
                elif self.curr_uncertainty_values[node_name] < initial_obstacle_prob:
                    self.curr_uncertainty_values[node_name] = self.curr_uncertainty_values[node_name] + (
                                initial_obstacle_prob / observation_memory_length)
                else:
                    continue
        return

    def update_observations_history(self, current_observed_node_names):
        self.observations_history.append(current_observed_node_names)

    def forget_observations_history(self):
        self.observations_history = collections.deque(maxlen=observation_memory_length)
