"""
This class is supposed to capture everything for our duckietown env
"""
__all__ = [
    'DuckieTown',
]
import duckietown_uplan.graph_utils.segmentify as segmentify
from duckietown_uplan.graph_utils.augmentation import GraphAugmenter
import duckietown_world as dw
import geometry as geo
from duckietown_uplan.environment.duckie import Duckie
from duckietown_uplan.environment.constant import Constants as CONSTANTS
from duckietown_uplan.environment.utils import draw_graphs, create_graph_from_polygon, \
    is_point_in_bounding_box, create_graph_from_path, get_closest_neighbor, is_bounding_boxes_intersect, \
    create_graph_from_nodes
from random import randint
from duckietown_uplan.environment.footprint_table import FootprintTable
from duckietown_world.geo.transforms import SE2Transform
import numpy as np
import time


class DuckieTown(object):
    def __init__(self, map):
        self.original_map = map
        self.tile_size = map.tile_size
        self.skeleton_graph = dw.get_skeleton_graph(map)
        self.current_graph = self.skeleton_graph.G
        self.duckie_citizens = []
        self.current_occupied_nodes = []
        self.node_to_index = {}
        self.index_to_node = {}
        for i, name in enumerate(self.current_graph):
            self.node_to_index[name] = i
            self.index_to_node[i] = name
        self.collision_matrix = self._build_collision_matrix()
        #build the clustered graph with a random duckie
        random_node, _ = self.get_random_node_in_graph()
        max_radius = Duckie(-1,
                              CONSTANTS.duckie_width,
                              CONSTANTS.duckie_height,
                              velocity=0.25,
                              position=random_node['point']).get_max_radius()
        foot_print_table = FootprintTable(self.current_graph, max_radius)
        self.clustered_graph = foot_print_table.get_data()

    def get_map_original_graph(self):
        return dw.get_skeleton_graph(self.original_map).G

    def get_current_graph(self):
        return self.current_graph

    def augment_graph(self):
        self.skeleton_graph = segmentify.get_skeleton_graph(self.original_map)  # to be changed accordig to Jose
        self.current_graph = GraphAugmenter.augment_graph(self.skeleton_graph.G,
                                                          num_long=0,
                                                          num_right=1,
                                                          num_left=0,
                                                          lat_dist=0.3)
        self.node_to_index = {}
        self.index_to_node = {}
        for i, name in enumerate(self.current_graph):
            self.node_to_index[name] = i
            self.index_to_node[i] = name
        self.collision_matrix = self._build_collision_matrix()
        # build the clustered graph with a random duckie
        random_node, _ = self.get_random_node_in_graph()
        max_radius = Duckie(-1,
                            CONSTANTS.duckie_width,
                            CONSTANTS.duckie_height,
                            velocity=0.25,
                            position=random_node['point']).get_max_radius()
        foot_print_table = FootprintTable(self.current_graph, max_radius)
        self.clustered_graph = foot_print_table.get_data()
        return

    def get_map(self):
        return self.original_map

    def get_random_node_in_graph(self):
        random_num = randint(0, len(self.index_to_node)-1)
        node_name = self.index_to_node[random_num]
        node = self.current_graph.nodes(data=True)[node_name]
        return node, node_name

    def render_current_graph(self, save=False, folder='.', file_index=None, display=False):
        # create a graph from each duckiebot
        final_graphs = [self.current_graph]
        node_colors = ['pink']
        edge_colors = ['pink']
        for duckie in self.duckie_citizens:
            final_graphs.append(create_graph_from_polygon(duckie.get_duckie_bounding_box()))
            node_colors.append('black')
            edge_colors.append('black')
            final_graphs.append(create_graph_from_polygon(duckie.get_field_of_view()))
            node_colors.append('blue')
            edge_colors.append('blue')
            #add path if visible
            if duckie.has_visible_path:
                final_graphs.append(create_graph_from_path(duckie.get_path_SE2()))
                node_colors.append('red')
                edge_colors.append('red')
        #draw blocked nodes
        final_graphs.append(create_graph_from_nodes(self.draw_blocked_nodes()))
        node_colors.append('purple')
        edge_colors.append('purple')
        for duckie in self.duckie_citizens:
            if duckie.has_visible_path:
                final_graphs.append(create_graph_from_nodes(duckie.get_current_fov_occupancy_graph()))
                node_colors.append('green')
                edge_colors.append('green')
        draw_graphs(final_graphs, with_labels=False, node_colors=node_colors,
                    edge_colors=edge_colors, save=save, folder=folder, file_index=file_index,
                    display=display)

    def draw_map_with_lanes(self):
        from duckietown_world.svg_drawing.ipython_utils import ipython_draw_html
        ipython_draw_html(self.skeleton_graph.root2)
        return

    def spawn_duckie(self, SE2_location):
        new_duckie = Duckie(len(self.duckie_citizens),
                            CONSTANTS.duckie_width,
                            CONSTANTS.duckie_height,
                            velocity=0.25,
                            position=SE2_location)
        for duckie in self.duckie_citizens:
            if is_bounding_boxes_intersect(duckie.get_duckie_safe_bounding_box(),
                                           new_duckie.get_duckie_safe_bounding_box()):
                print("The duckie you are trying to spawn doesn't make duckietown safer, it collides with others")
                return
        new_duckie.map_environment(self.current_graph,
                                   self.node_to_index,
                                   self.index_to_node,
                                   self.collision_matrix)
        self.duckie_citizens.append(new_duckie)
        self.update_blocked_nodes()
        return

    def spawn_random_duckie(self, num_of_duckies):
        #pick random control point that is not blocked
        """
        TODO: pick random control point that is not blocked
        """
        for i in range(num_of_duckies):
            new_duckie = None
            while new_duckie is None:
                random_node, _ = self.get_random_node_in_graph()
                new_duckie = Duckie(len(self.duckie_citizens),
                                    CONSTANTS.duckie_width,
                                    CONSTANTS.duckie_height,
                                    velocity=0.25,
                                    position=random_node['point'])
                for duckie in self.duckie_citizens:
                    if is_bounding_boxes_intersect(duckie.get_duckie_safe_bounding_box(),
                                                   new_duckie.get_duckie_safe_bounding_box()):
                        new_duckie = None
                        break
            new_duckie.map_environment(self.current_graph,
                                       self.node_to_index,
                                       self.index_to_node,
                                       self.collision_matrix)#takes a lot of time for now TODO: need to be optimized
            self.duckie_citizens.append(new_duckie)
        self.update_blocked_nodes()
        return

    def get_duckie_citizens(self):
        return self.duckie_citizens

    def get_duckie(self, duckie_id):
        return self.duckie_citizens[duckie_id]

    def get_duckie_foot_print(self, duckie_id):
        foot_print = []
        #replacing looping over nodes to looping over cluster
        # for node in self.current_graph.nodes(data=True):
        #     node_data = node[1]
        #     if is_point_in_bounding_box(node_data['point'],
        #                                 self.duckie_citizens[duckie_id].get_duckie_bounding_box()):
        #         foot_print.append(node)
        closest_node = self.duckie_citizens[duckie_id].get_closest_node()
        for node_name in self.clustered_graph[closest_node]:
            node_data = self.current_graph.nodes(data=True)[node_name]
            if is_point_in_bounding_box(node_data['point'],
                                        self.duckie_citizens[duckie_id].get_duckie_bounding_box()):
                foot_print.append((node_name, node_data))
        return foot_print

    def get_duckie_safe_foot_print(self, duckie_id):
        safe_foot_print = []
        closest_node = self.duckie_citizens[duckie_id].get_closest_node()
        for node_name in self.clustered_graph[closest_node]:
            node_data = self.current_graph.nodes(data=True)[node_name]
            if is_point_in_bounding_box(node_data['point'],
                                        self.duckie_citizens[duckie_id].get_duckie_safe_bounding_box()):
                safe_foot_print.append((node_name, node_data))
        return safe_foot_print

    def get_duckie_current_frame(self, duckie_id):
        # get observed duckies
        observed_duckies = []
        for duckie in self.duckie_citizens:
            if duckie != self.duckie_citizens[duckie_id] and is_bounding_boxes_intersect(duckie.get_duckie_bounding_box(), self.duckie_citizens[duckie_id].get_field_of_view()):
                observed_duckies.append(duckie)
        # get observed nodes
        observed_nodes = []
        closest_node = self.duckie_citizens[duckie_id].get_closest_node()
        for node_name in self.clustered_graph[closest_node]:
            node_data = self.current_graph.nodes(data=True)[node_name]
            if is_point_in_bounding_box(node_data['point'], self.duckie_citizens[duckie_id].get_field_of_view()):
                observed_nodes.append((node_name, node_data))
        return observed_duckies, observed_nodes

    def issue_ticket(self, duckie_id):
        raise Exception('DuckieTown issue_ticket not implemented')

    def retrieve_tickets(self):
        raise Exception('DuckieTown retrieve_tickets not implemented')

    def update_blocked_nodes(self):
        self.current_occupied_nodes = []
        for duckie in self.duckie_citizens:
            self.current_occupied_nodes.extend(self.get_duckie_safe_foot_print(duckie.id))
        return

    def draw_blocked_nodes(self):
        nodes_SE2 = [occupied_node[1]['point'] for occupied_node in self.current_occupied_nodes]
        print('current occupied nodes are ', nodes_SE2)
        return nodes_SE2

    def step(self, time_in_seconds, display=False, save=False, folder='./data', file_index=0):
        for duckie in self.duckie_citizens:
            if not duckie.is_stationary():
                time1 = time.time()
                duckie.move(time_in_seconds)
                time2 = time.time()
                observed_duckies, observed_nodes = self.get_duckie_current_frame(duckie.id)
                time3 = time.time()
                foot_print = self.get_duckie_foot_print(duckie.id)
                time4 = time.time()
                safe_foot_print = self.get_duckie_safe_foot_print(duckie.id)
                time5 = time.time()
                duckie.set_current_frame(observed_duckies, observed_nodes)
                time6 = time.time()
                duckie.set_foot_print(foot_print)
                time7 = time.time()
                duckie.set_safe_foot_print(safe_foot_print)
                time8 = time.time()
        self.update_blocked_nodes()
        if display or save:
            self.render_current_graph(display=display,
                                      save=save, folder=folder,
                                      file_index=file_index)
        return

    def reset(self, display=False, folder='./data', file_index=0):
        self.current_occupied_nodes = []
        for duckie in self.duckie_citizens:
            observed_duckies, observed_nodes = self.get_duckie_current_frame(duckie.id)
            foot_print = self.get_duckie_foot_print(duckie.id)
            safe_foot_print = self.get_duckie_safe_foot_print(duckie.id)
            duckie.set_current_frame(observed_duckies, observed_nodes)
            duckie.set_foot_print(foot_print)
            duckie.set_safe_foot_print(safe_foot_print)
            self.update_blocked_nodes()
        if display:
            self.render_current_graph(save=True, folder=folder, file_index=file_index)
        return

    def create_random_targets_for_all_duckies(self):
        for duckie in self.duckie_citizens:
            if duckie.is_stationary():
                _, random_end_node_name = self.get_random_node_in_graph()
                duckie.set_target_destination(random_end_node_name)
        return

    def is_duckie_violating(self, duckie):
        raise Exception('DuckieTown is_duckie_violating not implemented')

    def _build_collision_matrix(self):
        from shapely.geometry.polygon import Polygon

        def get_bounding_box(center_point):
            # BB ll, lr, ul, ur
            bounding_box = []
            # lower_right
            relative_transform = geo.SE2_from_translation_angle([-CONSTANTS.duckie_width / 2, -CONSTANTS.duckie_height / 2], 0)
            transform = geo.SE2.multiply(center_point.as_SE2(), relative_transform)
            bounding_box.append(SE2Transform.from_SE2(transform))
            # lower_left
            relative_transform = geo.SE2_from_translation_angle([CONSTANTS.duckie_width / 2, -CONSTANTS.duckie_height / 2], 0)
            transform = geo.SE2.multiply(center_point.as_SE2(), relative_transform)
            bounding_box.append(SE2Transform.from_SE2(transform))
            # upper left
            relative_transform = geo.SE2_from_translation_angle([CONSTANTS.duckie_width / 2, CONSTANTS.duckie_height / 2], 0)
            transform = geo.SE2.multiply(center_point.as_SE2(), relative_transform)
            bounding_box.append(SE2Transform.from_SE2(transform))
            # upper right
            relative_transform = geo.SE2_from_translation_angle([-CONSTANTS.duckie_width / 2, CONSTANTS.duckie_height / 2], 0)
            transform = geo.SE2.multiply(center_point.as_SE2(), relative_transform)
            bounding_box.append(SE2Transform.from_SE2(transform))

            return bounding_box

        # def local_to_global(poly_points, q):
        #     global_points = []
        #     for points in poly_points:
        #         q_point = geo.SE2_from_translation_angle(points, 0.0)
        #         global_point, _ = geo.translation_angle_from_SE2(geo.SE2.multiply(q, q_point))
        #         global_points.append(global_point)
        #
        #     return global_points

        def overlap(q1, q2):
            # poly_points = ([((CONSTANTS.duckie_height / 2.0), (-CONSTANTS.duckie_width / 2.0)),
            #                 ((CONSTANTS.duckie_height / 2.0), (CONSTANTS.duckie_width / 2.0)),
            #                 ((-CONSTANTS.duckie_height / 2.0), (CONSTANTS.duckie_width / 2.0)),
            #                 ((-CONSTANTS.duckie_height / 2.0), (-CONSTANTS.duckie_width / 2.0))])
            return 1 if is_bounding_boxes_intersect(get_bounding_box(q1), get_bounding_box(q2)) else 0

        num_nodes = len(self.current_graph)
        collision_matrix = [[0 for _ in range(num_nodes)] for _ in range(num_nodes)]

        for i in range(num_nodes):
            curr = self.current_graph.nodes[self.index_to_node[i]]['point']
            collision_matrix[i][i] = 1
            for j in range(i+1, num_nodes):
                q = self.current_graph.nodes[self.index_to_node[j]]['point']
                collision = overlap(curr, q)
                collision_matrix[i][j] = collision
                collision_matrix[j][i] = collision

        return np.array(collision_matrix)
