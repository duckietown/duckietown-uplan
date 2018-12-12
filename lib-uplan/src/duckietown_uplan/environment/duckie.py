"""
This class is supposed to capture everything for our duckiebot itself
"""
__all__ = [
    'Duckie',
]
import geometry as geo
from duckietown_world.geo.transforms import SE2Transform
import networkx as nx
from duckietown_uplan.environment.utils import transform_point, move_point, euclidean_distance


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
            if distance_to_next_ctrl_pt >= (distance_to_travel - distance_travelled):
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

    def stop_movement(self):
        self.motor_off = True
        return

    def is_stationary(self):
        return self.motor_off

    def retrieve_current_frame(self):
        raise Exception('Duckie retrieve current_frame not implemented')

    """
    This function is only used for the simulation purposes,
    we won't need to set the current_frame when deploying on the duckiebot
    """
    def set_current_frame(self, global_duckie_positions, map_graph):
        raise Exception('Duckie set current_frame not implemented')

    def _get_field_of_view(self):
        # BB ll, lr, ul, ur
        bounding_box = []
        # lower_right
        relative_transform = geo.SE2_from_translation_angle([self.size_x / 2, 0], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # lower_left
        relative_transform = geo.SE2_from_translation_angle([-self.size_x/2, 0], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper left
        relative_transform = geo.SE2_from_translation_angle([-self.size_x / 2, self.size_y], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))
        # upper right
        relative_transform = geo.SE2_from_translation_angle([self.size_x / 2, self.size_y], 0)
        transform = geo.SE2.multiply(self.current_position.as_SE2(), relative_transform)
        bounding_box.append(SE2Transform.from_SE2(transform))

        return bounding_box

    def _create_graph_from_polygon(self, polygon_nodes):
        bb_graph = nx.MultiDiGraph()
        for i in range(len(polygon_nodes)):
            bb_graph.add_node('node_'+str(i), point=polygon_nodes[i])
        #create edges now
        for i in range(len(polygon_nodes) - 1):
            bb_graph.add_edge('node_'+str(i), 'node_'+str(i + 1))
        bb_graph.add_edge('node_' + str(len(polygon_nodes)-1), 'node_' + str(0))

        return bb_graph





