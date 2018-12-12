"""
Each duckiebot will have its own observation model to update it
"""
__all__ = [
    'ObservationModel',
]


class ObservationModel(object):
    def __init__(self, map):
        raise Exception("Observation Model __init__ not implemented yet")

    def reset_obstacles_uncertainity(self):
        raise Exception("Observation Model reset_obstacles_uncertainity not implemented yet")

    def get_observations_from_frame(self):
        raise Exception("Observation Model get_observations_from_frame not implemented yet")

    def update_obstacles_uncertainity(self):
        raise Exception("Observation Model update_obstacles_uncertainity not implemented yet")

    def update_observations_history(self):
        raise Exception("Observation Model update_observations_history not implemented yet")

    def forget_observations_history(self):
        raise Exception("Observation Model forget_observations_history not implemented yet")