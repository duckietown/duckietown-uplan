import geometry as geo
from duckietown_world.geo.transforms import SE2Transform
import numpy as np
import copy


def transform_point(location_SE2, dx, dy, dtheta):
    relative_transform = geo.SE2_from_translation_angle([dx, dy], dtheta)
    transform = geo.SE2.multiply(location_SE2.as_SE2(), relative_transform)
    return SE2Transform.from_SE2(transform)


def move_point(location_SE2, distance, theta):
    new_location = copy.deepcopy(location_SE2)
    dp = [distance*-np.sin(theta), distance*np.cos(theta)]
    new_location.p = [(new_location.p[0] + dp[0]), (new_location.p[1] + dp[1])]
    return new_location


def euclidean_distance(self, node1_SE2, node2_SE2):
    return np.linalg.norm(node1_SE2.p-node2_SE2.p)


def is_point_in_bounding_box(point_SE2, bb):
    from shapely.geometry import Point
    from shapely.geometry.polygon import Polygon
    point = point_SE2.p
    point = Point(point)
    polygon_points = [bb_p.p for bb_p in bb]
    polygon = Polygon(polygon_points)
    return polygon.contains(point)
