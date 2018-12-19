import geometry as geo
from duckietown_world.geo.transforms import SE2Transform
import numpy as np
import copy
import networkx as nx


def transform_point(location_SE2, dx, dy, dtheta):
    relative_transform = geo.SE2_from_translation_angle([dx, dy], dtheta)
    transform = geo.SE2.multiply(location_SE2.as_SE2(), relative_transform)
    return SE2Transform.from_SE2(transform)


def move_point(location_SE2, distance, theta):
    new_location = copy.deepcopy(location_SE2)
    dp = [distance*np.cos(theta), distance*np.sin(theta)]
    new_location.theta = theta
    new_location.p = np.array([(new_location.p[0] + dp[0]), (new_location.p[1] + dp[1])], dtype='float64')
    return new_location


def euclidean_distance(node1_SE2, node2_SE2):
    return np.linalg.norm(node1_SE2.p-node2_SE2.p)


def is_point_in_bounding_box(point_SE2, bb):
    from shapely.geometry import Point
    from shapely.geometry.polygon import Polygon
    point = point_SE2.p
    point = Point(point)
    polygon_points = [bb_p.p for bb_p in bb]
    polygon = Polygon(polygon_points)
    return polygon.contains(point)


def is_bounding_boxes_intersect(bb1, bb2):
    from shapely.geometry import Point
    from shapely.geometry.polygon import Polygon
    polygon_points_1 = [bb_p.p for bb_p in bb1]
    polygon_points_2 = [bb_p.p for bb_p in bb2]
    polygon_1 = Polygon(polygon_points_1)
    polygon_2 = Polygon(polygon_points_2)
    return polygon_1.intersects(polygon_2)


def get_absolute_position_from_graph(graph):
    import geometry as geo
    pos = {}
    for n in graph:
        q = graph.nodes[n]['point'].as_SE2()
        t, _ = geo.translation_angle_from_SE2(q)
        pos[n] = t
    return pos


def draw_graphs(graphs, with_labels=False, node_colors=None, edge_colors=None, save=False,
                folder='../', file_index=None, display=False):
    import networkx as nx
    from matplotlib import pyplot as plt
    if node_colors is None:
        node_colors = ['red'] * len(graphs)

    if edge_colors is None:
        edge_colors = ['balck'] * len(graphs)

    plt.figure(figsize=(12, 12))
    for i in range(len(graphs)):
        curr_pos = get_absolute_position_from_graph(graphs[i])
        nx.draw(graphs[i], curr_pos, with_labels=with_labels, node_size=10, node_color=node_colors[i],
                edge_color=edge_colors[i])
    fig_to_save = plt.gcf()
    plt.axis('off')
    if display:
        plt.show()
        plt.draw()
    if save:
        fig_to_save.savefig(folder + "/file%02d.png" % file_index)


def create_graph_from_polygon(polygon_nodes):
    bb_graph = nx.MultiDiGraph()
    for i in range(len(polygon_nodes)):
        bb_graph.add_node('node_'+str(i), point=polygon_nodes[i])
    #create edges now
    for i in range(len(polygon_nodes) - 1):
        bb_graph.add_edge('node_'+str(i), 'node_'+str(i + 1))
    bb_graph.add_edge('node_' + str(len(polygon_nodes)-1), 'node_' + str(0))

    return bb_graph


def create_graph_from_path(path_nodes):
    bb_graph = nx.MultiDiGraph()
    for i in range(len(path_nodes)):
        bb_graph.add_node('node_'+str(i), point=path_nodes[i])
    #create edges now
    for i in range(len(path_nodes) - 1):
        bb_graph.add_edge('node_'+str(i), 'node_'+str(i + 1))

    return bb_graph


def create_graph_from_nodes(nodes):
    bb_graph = nx.MultiDiGraph()
    for i in range(len(nodes)):
        bb_graph.add_node('node_'+str(i), point=nodes[i])

    return bb_graph


def get_closest_neighbor(graph, node_location):
    closest_node_name = None
    min_distance = float('inf')
    for neighbor in graph.nodes(data=True):
        neighbor_name = neighbor[0]
        neighbor_location = neighbor[1]['point']
        curr_distance = euclidean_distance(node_location, neighbor_location)
        if curr_distance < min_distance:
            min_distance = curr_distance
            closest_node_name = neighbor_name
    return closest_node_name, min_distance


def interpolate(q0, q1, alpha):
    q1_from_q0 = geo.SE2.multiply(geo.SE2.inverse(q0), q1)
    vel = geo.SE2.algebra_from_group(q1_from_q0)
    rel = geo.SE2.group_from_algebra(vel * alpha)
    q = geo.SE2.multiply(q0, rel)
    return q