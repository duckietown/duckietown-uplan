"""
This class is supposed to capture everything for our duckiebot itself
"""
__all__ = [
    'Duckie',
]
import geometry as geo
from duckietown_world.geo.transforms import SE2Transform
from duckietown_uplan.environment.utils import move_point, euclidean_distance, \
    is_point_in_bounding_box, interpolate, get_closest_neighbor
from duckietown_uplan.environment.constant import Constants as CONSTANTS
from duckietown_uplan.algo.path_planning import PathPlanner
from duckietown_uplan.algo.velocity_profiling import VelocityProfiler
from duckietown_uplan.algo.observations import ObservationModel
import numpy as np
import collections


class Duckie(object):
    def __init__(self, id, size_x, size_y, velocity, position):
        self.id = id
        self.size_x = size_x
        self.size_y = size_y
        self.current_position = position
        self.velocity = velocity #cm/s
        self.current_path = [] #stack
        self.current_velocity_profile = []  # stack
        self.motor_off = False
        self.destination_node = None
        self.current_observed_nodes = None
        self.current_observed_duckies = None
        self.current_foot_print = None
        self.current_safe_foot_print = None
        self.has_visible_path = False
        self.env_graph = None
        self.path_planner = None
        self.velocity_profiler = None
        self.my_closest_control_point = None
        self.observation_model = None
        self.replan = False
        self.occupancy_vector = collections.deque(maxlen=10)

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
        self.velocity_profiler = VelocityProfiler(velocity_min=0.1, velocity_max=0.7, N=10)
        self.my_closest_control_point, _ = get_closest_neighbor(graph, self.current_position)
        self.observation_model = ObservationModel(graph)
        return

    def move(self, time_in_seconds):
        """
        TODO: take in consideration smooth turns and case where the duckie is not exactly on a trajectory
        TODO: currently assuming that duckies are always on a path
        """
        print("Started move function")
        self.observation_model.update_obstacles_uncertainity(self.get_current_observations())

        path_nodes = [path_node[1]['point'] for path_node in self.current_path]
        if len(self.current_path) > 0 and (self.current_position == path_nodes[0].as_SE2()).all():
            my_closest_control_point = self.current_path.pop(0)
            self.current_velocity_profile.pop(0)
            self.my_closest_control_point = my_closest_control_point[0]

        if len(self.current_path) == 0:
            return
        #replan only when above a control point
        if self.replan:
            self.replan = False
            print('replanning now')
            self.current_path = self.path_planner.get_shortest_path(self.my_closest_control_point,
                                                                    self.destination_node,
                                                                    self.get_fov_occupancy())
            self.current_velocity_profile = self.velocity_profiler.get_velocity_profile(self.velocity,
                                                                                        self.current_path,
                                                                                        self.observation_model.get_path_uncertainities(self.current_path))
            path_nodes = [path_node[1]['point'] for path_node in self.current_path]
            if len(self.current_path) == 0:
                return

        if self.motor_off:
            return
        num_alpha = 20
        steps = np.linspace(0, 1, num_alpha)
        q0 = self.current_position.as_SE2()
        seqs = []
        for cp in path_nodes:
            q1 = cp.as_SE2()
            q1_se2_pos, q1_se2_theta = geo.translation_angle_from_SE2(q1)
            q1_se2 = SE2Transform(q1_se2_pos, q1_se2_theta)
            q0_se2_pos, q0_se2_theta = geo.translation_angle_from_SE2(q0)
            q0_se2 = SE2Transform(q0_se2_pos, q0_se2_theta)
            if q0_se2.theta == q1_se2.theta and q0_se2.p[0] != q1_se2.p[0] and q0_se2.p[1] != q1_se2.p[1]:
                # q0_se2.theta = q0_se2.theta - np.arctan2(q0_se2.p[1] - q1_se2.p[1], q0_se2.p[0] - q1_se2.p[0])
                q0_se2.theta = q0_se2.theta
                q0 = q0_se2.as_SE2()
            sub_seq = []
            for alpha in steps:
                q = interpolate(q0, q1, alpha)
                sub_seq.append(q)
            seqs.extend(sub_seq[1:])
            q0 = q1

        self.velocity = self.current_velocity_profile[0]
        distance_to_travel = self.velocity * time_in_seconds

        dist = 0

        old_pose = self.current_position.as_SE2()
        for pose in seqs:
            if dist >= distance_to_travel:
                break
            p_old, theta_old = geo.translation_angle_from_SE2(old_pose)
            p_pose, theta_pose = geo.translation_angle_from_SE2(pose)
            dist += euclidean_distance(SE2Transform(p_old, theta_old), SE2Transform(p_pose, theta_pose))
            # dist += geo.SE2.distances(old_pose, pose)[1]
            if (np.isclose(path_nodes[0].as_SE2(), pose)).all():
                #possible bug here, list index out of range sometimes
                path_nodes.pop(0)
                my_closest_control_point = self.current_path.pop(0)
                self.current_velocity_profile.pop(0)
                if len(self.current_velocity_profile) > 0:
                    delta_t = time_in_seconds - (dist / self.velocity)
                    distance_to_travel = delta_t * self.current_velocity_profile[0]
                    dist = 0
                    self.velocity = self.current_velocity_profile[0]
                self.my_closest_control_point = my_closest_control_point[0]
                self.replan = True
            old_pose = pose

        p, theta = geo.translation_angle_from_SE2(old_pose)

        self.current_position = SE2Transform(p, theta)
        return

    def get_current_positon(self):
        return self.current_position

    def set_current_positon(self, position):
        self.current_position = position
        # self.my_closest_control_point, _ = get_closest_neighbor(self.env_graph, self.current_position)
        return

    def get_closest_node(self):
        return self.my_closest_control_point

    def set_path(self, path):
        self.current_path = path
        return

    def get_path(self):
        return self.current_path

    def get_path_SE2(self):
        return [node_path[1]['point'] for node_path in self.current_path]

    def get_max_radius(self):
        dif = self.current_position.p - self.get_field_of_view()[-1].p
        offset = self.current_position.p - self.get_duckie_bounding_box()[1].p  #lower left corner
        return np.linalg.norm(dif) + np.linalg.norm(offset)

    def append_path(self, path):
        self.current_path.extend(path)
        return

    def get_current_fov_occupancy(self):
        occupied_nodes = []
        for duckie in self.current_observed_duckies:
            #get duckie footprint and get the nodes that I can see
            for duckie_foot_print_node in duckie.current_safe_foot_print:
                if duckie_foot_print_node in self.current_observed_nodes:
                    #print("Duckies colliding yaaaay")
                    occupied_nodes.append(duckie_foot_print_node[0])
        return occupied_nodes

    def get_fov_occupancy(self):
        curren_fov_occupancy = self.get_current_fov_occupancy()
        self.occupancy_vector.append(curren_fov_occupancy)
        if len(curren_fov_occupancy) > 0:
            print("debug")
        return [item for sublist in self.occupancy_vector for item in sublist]

    def get_current_fov_occupancy_graph(self):
        occupied_nodes = []
        for duckie in self.current_observed_duckies:
            #get duckie footprint and get the nodes that I can see
            for duckie_foot_print_node in duckie.current_safe_foot_print:
                if duckie_foot_print_node in self.current_observed_nodes:
                    occupied_nodes.append(duckie_foot_print_node[1]['point'])
        return occupied_nodes

    def set_target_destination(self, destination_node):
        self.destination_node = destination_node
        self.current_path = self.path_planner.get_shortest_path(self.my_closest_control_point,
                                                                self.destination_node)
        self.current_velocity_profile = self.velocity_profiler.get_velocity_profile(self.velocity,
                                                                                    self.current_path,
                                                                                    self.observation_model.get_path_uncertainities(self.current_path))
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
        if len(self.current_observed_duckies):
            self.replan = True
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
        safe_dist = 0.02
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

    def get_current_observations(self):
        fov_occupancy_nodes = self.get_current_fov_occupancy()
        observations = {}
        for node in self.current_observed_nodes:
            node_name = node[0]
            observations[node_name] = 0
        for node_name in fov_occupancy_nodes:
            observations[node_name] = 1
        return observations
