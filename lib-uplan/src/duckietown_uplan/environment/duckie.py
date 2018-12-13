"""
This class is supposed to capture everything for our duckiebot itself
"""
__all__ = [
    'Duckie',
]
import geometry as geo
from duckietown_world.geo.transforms import SE2Transform
from duckietown_uplan.environment.utils import move_point, euclidean_distance, is_point_in_bounding_box
from duckietown_uplan.environment.constant import Constants as CONSTANTS



class Duckie(object):
    def __init__(self, id, size_x, size_y, velocity, position):
        self.id = id
        self.size_x = size_x
        self.size_y = size_y
        self.current_position = position
        self.velocity = velocity #cm/s
        self.current_path = [] #stack
        self.motor_off = False
        self.current_observed_nodes = None
        self.current_observed_duckies = None
        self.has_visible_path = False

    def move(self, time_in_seconds):
        """
        TODO: take in consideration smooth turns and case where the duckie is not exactly on a trajectory
        TODO: currently assuming that duckies are always on a path
        """
        if self.motor_off:
            return
        distance_to_travel = self.velocity*time_in_seconds
        distance_travelled = 0
        moving_position = self.current_position
        while(distance_travelled < distance_to_travel):
            #pick next control point
            target_node = self.current_path[0]
            #measure euclidean distance
            distance_to_next_ctrl_pt = euclidean_distance(target_node, moving_position)
            if distance_to_next_ctrl_pt <= (distance_to_travel - distance_travelled):
                #pop the control point and go on next one
                print('going to target_node')
                moving_position = target_node
                distance_travelled = distance_travelled + distance_to_next_ctrl_pt
                self.current_path.pop(0)
                continue
            else:
                print('going to move_point')
                #go somewhere close to the next control point
                moving_position = move_point(location_SE2=moving_position,
                                             distance=distance_to_travel - distance_travelled,
                                             theta=target_node.theta)
                distance_travelled = distance_to_travel

        self.current_position = moving_position
        print(self.current_position)
        return

    def get_current_positon(self):
        return self.current_position

    def set_current_positon(self, position):
        self.current_position = position
        return

    def set_path(self, path):
        self.current_path = path
        return

    def get_path(self):
        return self.current_path

    def append_path(self, path):
        self.current_path.extend(path)
        return

    def stop_movement(self):
        self.motor_off = True
        return

    def is_stationary(self):
        return self.motor_off

    def retrieve_observed_duckies(self):
        return [duckie.id for duckie in self.current_observed_duckies]

    def retrieve_observed_nodes(self):
        return self.current_observed_nodes

    def set_visible_path(self, value):
        self.has_visible_path = value

    """
    This function is only used for the simulation purposes,
    we won't need to set the current_frame when deploying on the duckiebot
    """
    def set_current_frame(self, global_duckies, map_graph):
        #get observed duckies
        self.current_observed_duckies = []
        for duckie in global_duckies:
            if is_point_in_bounding_box(duckie.current_position, self.get_field_of_view()):
                self.current_observed_duckies.append(duckie)
        #get observed nodes
        self.current_observed_nodes = []
        for node in map_graph.nodes(data=True):
            node_data = node[1]
            if is_point_in_bounding_box(node_data['point'], self.get_field_of_view()):
                self.current_observed_nodes.append(node)
        return

    def get_field_of_view(self):
        # BB ll, lr, ul, ur
        bounding_box = []
        # lower_right
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.duckie_width/2, CONSTANTS.FOV_horizontal], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # lower_left
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.duckie_width/2, -CONSTANTS.FOV_horizontal], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper left
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.FOV_vertical + CONSTANTS.duckie_width/2,
                                                             -CONSTANTS.FOV_horizontal], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper right
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.FOV_vertical + CONSTANTS.duckie_width/2,
                                                             CONSTANTS.FOV_horizontal], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))

        return bounding_box

    def get_duckie_bounding_box(self):
        # BB ll, lr, ul, ur
        bounding_box = []
        # lower_right
        relative_transform = geo.SE2_from_translation_angle([-CONSTANTS.duckie_width/2, -CONSTANTS.duckie_height/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # lower_left
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.duckie_width/2, -CONSTANTS.duckie_height/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper left
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.duckie_width/2, CONSTANTS.duckie_height/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper right
        relative_transform = geo.SE2_from_translation_angle([-CONSTANTS.duckie_width/2, CONSTANTS.duckie_height/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))

        return bounding_box







