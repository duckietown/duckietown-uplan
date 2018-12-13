"""
This class is supposed to capture everything for our duckietown env
"""
__all__ = [
    'DuckieTown',
]
import duckietown_uplan.graph_utils.segmentify as segmentify
import duckietown_uplan.graph_utils.augmentation as augmentify
import duckietown_world as dw
from duckietown_uplan.environment.duckie import Duckie
from duckietown_uplan.environment.constant import Constants as CONSTANTS
from duckietown_uplan.environment.utils import draw_graphs, create_graph_from_polygon, create_graph_from_path
from random import randint

class DuckieTown(object):
    def __init__(self, map):
        self.original_map = map
        self.tile_size = map.tile_size
        self.skeleton_graph = dw.get_skeleton_graph(map)
        self.current_graph = self.skeleton_graph.G
        self.duckie_citizens = []

    def get_map_original_graph(self):
        return dw.get_skeleton_graph(self.original_map).G

    def get_map_current_graph(self):
        return self.current_graph

    def augment_graph(self):
        self.skeleton_graph = segmentify.get_skeleton_graph(self.original_map)  # to be changed accordig to Jose
        self.current_graph = augmentify.augment_graph(self.skeleton_graph.G,
                                                      num_right=CONSTANTS.aug_num_right,
                                                      num_left=CONSTANTS.aug_num_left,
                                                      dist=CONSTANTS.aug_dist)
        return

    def get_map(self):
        return self.original_map

    def get_random_node_in_graph(self):
        keys = [node for node in self.current_graph.nodes]
        random_num = randint(0, len(keys))
        return self.current_graph.nodes[keys[random_num]], keys[random_num]

    def render_current_graph(self, save=False, folder='.', file_index=None):
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
                final_graphs.append(create_graph_from_path(duckie.get_path()))
                node_colors.append('red')
                edge_colors.append('red')
        draw_graphs(final_graphs, with_labels=False, node_colors=node_colors,
                    edge_colors=edge_colors, save=save, folder=folder, file_index=file_index)

    def draw_map_with_lanes(self):
        from duckietown_world.svg_drawing.ipython_utils import ipython_draw_html
        ipython_draw_html(self.skeleton_graph.root2)
        return

    def spawn_duckie(self, SE2_location):
        self.duckie_citizens.append(Duckie(len(self.duckie_citizens),
                                           CONSTANTS.duckie_width,
                                           CONSTANTS.duckie_height,
                                           velocity=0.01,
                                           position=SE2_location))
        return

    def spawn_random_duckie(self, num_of_duckies):
        #pick random control point that is not blocked
        """
        TODO: pick random control point that is not blocked
        """
        for _ in range(num_of_duckies):
            random_node, _ = self.get_random_node_in_graph()
            self.duckie_citizens.append(Duckie(len(self.duckie_citizens),
                                               CONSTANTS.duckie_width,
                                               CONSTANTS.duckie_height,
                                               velocity=0.2,
                                               position=random_node['point']))
        return

    def get_duckie_citizens(self):
        return self.duckie_citizens

    def get_duckie(self, id):
        return self.duckie_citizens[id]

    def issue_ticket(self, duckie_id):
        raise Exception('DuckieTown issue_ticket not implemented')

    def retrieve_tickets(self):
        raise Exception('DuckieTown retrieve_tickets not implemented')

    def get_blocked_nodes(self):
        raise Exception('DuckieTown get_blocked_nodes not implemented')

    def step(self, time_in_seconds):
        raise Exception('DuckieTown step not implemented')

    def reset(self):
        raise Exception('DuckieTown reset not implemented')

    def is_duckie_violating(self, duckie):
        raise Exception('DuckieTown is_duckie_violating not implemented')