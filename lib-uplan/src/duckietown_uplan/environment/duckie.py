"""
This class is supposed to capture everything for our duckiebot itself
"""
__all__ = [
    'Duckie',
]


class Duckie(object):
    def __init__(self):
        raise Exception('Duckie init not implemented')

    def move(self):
        raise Exception('Duckie move not implemented')

    def get_current_positon(self):
        raise Exception('Duckie get current position not implemented')

    def set_current_positon(self):
        raise Exception('Duckie get current position not implemented')

    def set_path(self):
        raise Exception('Duckie set path not implemented')

    def get_path(self):
        raise Exception('Duckie get path not implemented')

    def stop_movement(self):
        raise Exception('Duckie stop_movement not implemented')

    def is_stationary(self):
        raise Exception('Duckie is_stationary not implemented')

    def retrieve_current_frame(self):
        raise Exception('Duckie retrieve current_frame not implemented')

    """
    This function is only used for the simulation purposes,
    we won't need to set the current_frame when deploying on the duckiebot
    """
    def set_current_frame(self):
        raise Exception('Duckie set current_frame not implemented')