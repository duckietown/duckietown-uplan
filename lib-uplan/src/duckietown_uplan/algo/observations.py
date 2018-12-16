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
        # Queue of nodes observed in the recent past
        self.uncertainty_history = collections.deque(maxlen=observation_memory_length)

    def reset_obstacles_uncertainity(self):
        for key in curr_uncertainty_values.keys():
            curr_uncertainty_values[key] = initial_obstacle_prob

    def get_uncertainty_from_node(self, node):
        return self.curr_uncertainty_values[node]

    def update_obstacles_uncertainity(self):
        from duckietown_uplan.environment.duckie import Duckie
        for key, value in Duckie.get_current_observations(): ## Returns a dict()
            self.curr_uncertainty_values[key] = value
        for n in self.uncertainty_history:
            if n in Duckie.get_current_observations():
                continue
            #reduce it if its higher than the initial obstacle prob and increase otherwise
            if self.curr_uncertainty_values[n] > initial_obstacle_prob:
                self.curr_uncertainty_values[n] = self.curr_uncertainty_values[n] * discount_factor
            elif self.curr_uncertainty_values[n] < initial_obstacle_prob:
                self.curr_uncertainty_values[n] = self.curr_uncertainty_values[n] + (initial_obstacle_prob / observation_memory_length)
            else:
                continue

        def update_observations_history(self, node):
            uncertainty_history.append(node)

        def forget_observations_history(self):
            raise Exception("Observation Model forget_observations_history not implemented yet")
