"""
This class is supposed to capture everything for our duckietown env
"""
__all__ = [
    'DuckieTown',
]


class DuckieTown(object):
    def __init__(self):
        raise Exception('DuckieTown init not implemented')

    def get_map_original_graph(self):
        raise Exception('DuckieTown get_map_original_graph not implemented')

    def get_map_augmented_graph(self):
        raise Exception('DuckieTown get_map_augmented_graph not implemented')

    def get_map(self):
        raise Exception('DuckieTown get_map not implemented')

    def get_blocked_nodes(self):
        raise Exception('DuckieTown get_blocked_nodes not implemented')

    def render_graph(self):
        raise Exception('DuckieTown render_graph not implemented')

    def draw_map_with_lanes(self):
        raise Exception('DuckieTown draw_map_with_lanes not implemented')

    def step(self, time_in_seconds):
        raise Exception('DuckieTown step not implemented')

    def reset(self):
        raise Exception('DuckieTown reset not implemented')

    def spawn_duckie(self, SE2_location):
        raise Exception('DuckieTown spawn_duckie not implemented')

    def get_duckie_citizens(self, SE2_location):
        raise Exception('DuckieTown spawn_duckie not implemented')

    def issue_ticket(self, duckie_id):
        raise Exception('DuckieTown issue_ticket not implemented')

    def retrieve_tickets(self):
        raise Exception('DuckieTown retrieve_tickets not implemented')

    def is_duckie_violating(self, duckie):
        raise Exception('DuckieTown is_duckie_violating not implemented')