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
from duckietown_uplan.algo.path_planning import PathPlanner


class Duckie(object):
    def __init__(self, id, size_x, size_y, velocity, position):
        self.id = id
        self.size_x = size_x
        self.size_y = size_y
        self.current_position = position
        self.velocity = velocity #cm/s
        self.current_path = [] #stack
        self.motor_off = False
        self.destination_node = None
        self.current_observed_nodes = None
        self.current_observed_duckies = None
        self.current_foot_print = None
        self.current_safe_foot_print = None
        self.has_visible_path = False
        self.env_graph = None
        self.path_planner = None

    def map_environment(self, graph, node_to_index, index_to_node, collision_matrix):
        #args need to be refactored
        self.env_graph = graph
        self.path_planner = PathPlanner(self.env_graph,
                                        length=self.size_y,
                                        width=self.size_x,
                                        node_to_index=node_to_index,
                                        index_to_node=index_to_node,
                                        collision_matrix=collision_matrix
                                        )
        return

    def move(self, time_in_seconds, replan=True):
        """
        TODO: take in consideration smooth turns and case where the duckie is not exactly on a trajectory
        TODO: currently assuming that duckies are always on a path
        """
        if replan:
            self.current_path = self.path_planner.get_shortest_path(self.current_position,
                                                                    self.destination_node,
                                                                    self.retrieve_observed_duckies_locs())

        if self.motor_off:
            return
        distance_to_travel = self.velocity*time_in_seconds
        distance_travelled = 0
        moving_position = self.current_position
        while(distance_travelled < distance_to_travel):
            #pick next control point
            if len(self.current_path) == 0:
                return
            target_node = self.current_path[0]
            #measure euclidean distance
            distance_to_next_ctrl_pt = euclidean_distance(target_node, moving_position)
            if distance_to_next_ctrl_pt <= (distance_to_travel - distance_travelled):
                #pop the control point and go on next one
                moving_position = target_node
                distance_travelled = distance_travelled + distance_to_next_ctrl_pt
                self.current_path.pop(0)
                continue
            else:
                #go somewhere close to the next control point
                moving_position = move_point(location_SE2=moving_position,
                                             distance=distance_to_travel - distance_travelled,
                                             theta=target_node.theta)
                distance_travelled = distance_to_travel

        self.current_position = moving_position
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

    def set_target_destination(self, destination_node):
        self.destination_node = destination_node
        return

    def stop_movement(self):
        self.motor_off = True
        return

    def is_stationary(self):
        return self.motor_off or (len(self.current_path) == 0)

    def retrieve_observed_duckies_locs(self):
        return [duckie.get_current_positon() for duckie in self.current_observed_duckies]

    def retrieve_observed_nodes(self):
        return self.current_observed_nodes

    def set_visible_path(self, value):
        self.has_visible_path = value

    def set_current_frame(self, observed_duckies, observed_nodes):
        self.current_observed_duckies = observed_duckies
        self.current_observed_nodes = observed_nodes
        return

    def set_foot_print(self, foot_print):
        self.current_foot_print = foot_print
        return

    def set_safe_foot_print(self, safe_foot_print):
        self.current_safe_foot_print = safe_foot_print
        return

    def get_field_of_view(self):
        # BB ll, lr, ul, ur
        bounding_box = []
        # lower_right
        relative_transform = geo.SE2_from_translation_angle([self.size_x/2, self.size_y/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # lower_left
        relative_transform = geo.SE2_from_translation_angle([self.size_x/2, -self.size_y/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper left
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.FOV_vertical + self.size_x/2,
                                                             -CONSTANTS.FOV_horizontal], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper right
        relative_transform = geo.SE2_from_translation_angle([CONSTANTS.FOV_vertical + self.size_x/2,
                                                             CONSTANTS.FOV_horizontal], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))

        return bounding_box

    def get_duckie_bounding_box(self):
        # BB ll, lr, ul, ur
        bounding_box = []
        # lower_right
        relative_transform = geo.SE2_from_translation_angle([-self.size_x/2, -self.size_y/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # lower_left
        relative_transform = geo.SE2_from_translation_angle([self.size_x/2, -self.size_y/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper left
        relative_transform = geo.SE2_from_translation_angle([self.size_x/2, self.size_y/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper right
        relative_transform = geo.SE2_from_translation_angle([-self.size_x/2, self.size_y/2], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))

        return bounding_box

    def get_duckie_safe_bounding_box(self):
        # BB ll, lr, ul, ur
        bounding_box = []
        # lower_right
        safe_dist = 0.01
        relative_transform = geo.SE2_from_translation_angle([-self.size_x/2 - safe_dist, -self.size_y/2 - safe_dist], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # lower_left
        relative_transform = geo.SE2_from_translation_angle([self.size_x/2 + safe_dist, -self.size_y/2 - safe_dist], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper left
        relative_transform = geo.SE2_from_translation_angle([self.size_x/2 + safe_dist, self.size_y/2 + safe_dist], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper right
        relative_transform = geo.SE2_from_translation_angle([-self.size_x/2 - safe_dist, self.size_y/2 + safe_dist], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))

        return bounding_box


