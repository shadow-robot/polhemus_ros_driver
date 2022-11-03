#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from math import cos, floor, pi, sin, sqrt
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d
from scipy.optimize import least_squares
import rosbag
import tf2_ros
# import tf2_py as tf2

import numpy
from geometry_msgs.msg import Point

import rospkg
import rospy

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

class SphereFit:
    def __init__(self, use_recorded_polhemus_data = None, data = None, plot = False) -> None:
        self._raw_data = []
        self._center = None
        self._radius = None
        self._best_candidate = None
        self.finger_polhemusstation_map = {'ff': 'polhemus_station_1', 'mf': 'polhemus_station_2', 'rf': 'polhemus_station_3', 'lf': 'polhemus_station_4'}
        if plot:
            self.setup_plot()

        if data:
            for point in data:
                self._raw_data.append(Point(point[0], point[1], point[2]))

        elif use_recorded_polhemus_data:
            # Open input and output rosbags
            POLHEMUS_DRIVER_PATH = rospack.get_path('polhemus_ros_driver')
            bag_file_path = f'glove_data{use_recorded_polhemus_data}'
            # bag_file_path = 'glove_data/Ethan/2022-11-01-12-27-32.bag'
            # bag_file_path = 'glove_data/Ethan/2022-11-01-12-27-45.bag'
            # bag_file_path = 'glove_data/Ethan/2022-11-01-12-27-56.bag'
            # bag_file_path = 'glove_data/Ethan/2022-11-01-12-28-08.bag'
            # bag_file_path = 'glove_data/Ethan/2022-11-01-12-28-28.bag'

            # bag_file_path = 'glove_data/Hugo/2022-11-01-13-49-17.bag'
            # bag_file_path = 'glove_data/Hugo/2022-11-01-13-49-30.bag'
            # bag_file_path = 'glove_data/Hugo/2022-11-01-13-49-45.bag'
            # bag_file_path = 'glove_data/Hugo/2022-11-01-13-49-55.bag'
            # bag_file_path = 'glove_data/Hugo/2022-11-01-13-50-09.bag'

            local_tf_buffer = tf2_ros.Buffer()
            # object_methods = [method_name for method_name in dir(local_tf_buffer)
            #       if callable(getattr(local_tf_buffer, method_name))]
            # print(object_methods)
            local_tf_buffer.clear()
            local_buffer_has_been_udpdated = False

            with rosbag.Bag(f'{POLHEMUS_DRIVER_PATH}/{bag_file_path}', 'r') as bag_file:
                for _, msg, _ in bag_file.read_messages(topics=['/tf']):
                    for individual_transform in msg.transforms:
                        # if individual_transform.header.frame_id == "polhemus_base_0":
                        if individual_transform.child_frame_id in ['polhemus_station_1', 'polhemus_station_2', 'polhemus_station_3', 'polhemus_station_4']:
                            # print(f'>>{individual_transform.header.frame_id}')
                            # print(individual_transform)
                            # m = geometry_msgs.msg.TransformStamped()
                            # m.header.frame_id = 'THISFRAME'
                            # m.child_frame_id = 'CHILD'
                            # m.transform.translation.x = 2.71828183
                            # m.transform.rotation.w = 1.0
                            local_tf_buffer.set_transform(individual_transform, "default_authority")
                            # rospy.sleep(2)
                            # print(local_tf_buffer._getFrameStrings())
                            ff_transform = local_tf_buffer.lookup_transform('polhemus_base_0', 'polhemus_station_1', rospy.Time(0))
                            # print(ff_transform)
                            local_buffer_has_been_udpdated = True

                    # For each TF message, add points related to 1 finger - choose between ff, mf, rf, or lf
                    if local_buffer_has_been_udpdated:
                        finger_transform = local_tf_buffer.lookup_transform('polhemus_base_0', self.finger_polhemusstation_map['ff'], rospy.Time(0))
                        point = Point()
                        point.x = finger_transform.transform.translation.x
                        point.y = finger_transform.transform.translation.y
                        point.z = finger_transform.transform.translation.z
                        self._raw_data.append(point)
                        local_buffer_has_been_udpdated = False

    def fit_sphere(self, min_coords, max_coords, min_radius, max_radius):
        # self._best_candidate = SphereFit.grid_vote(self._raw_data, min_coords, max_coords, min_radius, max_radius, 10)
        # result = least_squares(self.sphere_errors_optimizable, [0, 0, 0, 0.08], method='trf', bounds=(min_coords + [min_radius], max_coords + [max_radius]))
        # print("Results #1:")
        # print(f'{result.x[0]:.4f}\n{result.x[1]:.4f}\n{result.x[2]:.4f}\n{result.x[3]:.4f}')
        result = least_squares(self.sphere_errors_optimizable, [0, 0, 0, 0.08], loss="cauchy", bounds=(min_coords + [min_radius], max_coords + [max_radius]), f_scale=0.001)
        print("Results:")
        print(f'{result.x[0]:.4f}\n{result.x[1]:.4f}\n{result.x[2]:.4f}\n{result.x[3]:.4f}')
        self._best_candidate = result.x
        self._residuals = result.fun
        # result = least_squares(self.sphere_errors_optimizable, [0, 0, 0, 0.08], method="lm", f_scale=0.0001)
        # print("Results #3:")
        # print(f'{result.x[0]:.4f}\n{result.x[1]:.4f}\n{result.x[2]:.4f}\n{result.x[3]:.4f}')
        # result = least_squares(self.sphere_errors_optimizable, [0, 0, 0, 0.08], method='soft_l1', bounds=(min_coords + [min_radius], max_coords + [max_radius]), f_scale=0.1)
        # print(self._best_candidate)
        return self._best_candidate[3], self._best_candidate[0:3], self._residuals # radius, center, residuals

    @staticmethod
    def grid_vote(data, min_coords, max_coords, min_radius, max_radius, grid_points):
        x_candidates = SphereFit.inclusive_arange(min_coords[0], max_coords[0], grid_points)
        y_candidates = SphereFit.inclusive_arange(min_coords[1], max_coords[1], grid_points)
        z_candidates = SphereFit.inclusive_arange(min_coords[2], max_coords[2], grid_points)
        r_candidates = SphereFit.inclusive_arange(min_radius, max_radius, grid_points)
        r_error_threshold = abs(max_radius - min_radius) / (2 * (grid_points - 1))
        best_hypothesis = None
        best_score = 0
        for x in x_candidates:
            for y in y_candidates:
                for z in z_candidates:
                    for r in r_candidates:
                        errors = SphereFit.sphere_errors(data, Point(x, y, z), r)
                        votes = sum(map(lambda error: error <= r_error_threshold, errors))
                        if (votes > best_score):
                            best_score = votes
                            best_hypothesis = [x, y, z, r]
        return best_hypothesis
                    #    candidate_coords.append([x, y, z, r])

    @staticmethod
    def inclusive_arange(min, max, steps):
        arange = []
        step = (max - min) / (steps - 1)
        for i in range(steps - 1):
            arange.append(min + i * step)
        arange.append(max)
        print(arange)
        return arange


    @staticmethod
    def point_distance(point_1, point_2):
        return sqrt((point_1.x - point_2.x) ** 2 + (point_1.y - point_2.y) ** 2 + (point_1.z - point_2.z) ** 2)

    @staticmethod
    def sphere_errors(data, center, radius):
        errors = []
        for point in data:
            errors.append(abs(radius - SphereFit.point_distance(center, point)))
        return errors

    # @staticmethod
    def sphere_errors_optimizable(self, x):
        return SphereFit.sphere_errors(self._raw_data, Point(x[0], x[1], x[2]), x[3])
    
    def setup_plot(self):
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(projection='3d')
        self._ax.set_xlim3d(-0.1, 0.1)
        self._ax.set_ylim3d(0.0, 0.2)
        self._ax.set_zlim3d(-0.1, 0.1)
        self._ax.set_xlabel('X')
        self._ax.set_ylabel('Y')
        self._ax.set_zlabel('Z')

    def generate_data(self, N, center, radius, radius_std, polar_min, polar_max, azimuth_min, azimuth_max):
        self._radius = radius
        self._center = center
        radii = numpy.random.normal(radius, radius_std, size=N)
        polar = numpy.random.uniform(polar_min, polar_max, size=N)
        azimuth = numpy.random.uniform(azimuth_min, azimuth_max, size=N)
        self._raw_data = []
        for i in range(N):
            self._raw_data.append(SphereFit.point_from_polar(center, radii[i], polar[i], azimuth[i]))
        # Uncomment to add sparse noise
        # n_noise = floor(N/10)
        # for i in range(n_noise):
        #     self._raw_data.append(SphereFit.random_cartesian(center, [-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]))
        return self._raw_data

    @staticmethod
    def point_from_polar(center, radius, polar, azimuth):
        point = Point()
        point.x = center.x + radius * sin(polar) * cos(azimuth)
        point.y = center.y + radius * sin(polar) * sin(azimuth)
        point.z = center.z + radius * cos(polar)
        return point
    
    @staticmethod
    def random_cartesian(center, min, max):
        return Point(
            center.x + numpy.random.uniform(min[0], max[0]),
            center.y + numpy.random.uniform(min[1], max[1]),
            center.z + numpy.random.uniform(min[2], max[2]))

    def plot_data(self, points = None):
        if points is None:
            points = self._raw_data
        if self._residuals is None:
            self._ax.scatter([point.x for point in points], [point.y for point in points], [point.z for point in points], marker='.')
        else:
            c = (self._residuals - self._residuals.min()) / (self._residuals.max() - self._residuals.min())
            # print(c.min())
            # print(c.max())
            self._ax.scatter([point.x for point in points], [point.y for point in points], [point.z for point in points], marker='.', c=c, cmap="jet")
        if (self._center is not None):
            self._ax.scatter(self._center.x, self._center.y, self._center.z, c='r', marker='.')
        if (self._best_candidate is not None):
            self._ax.scatter(self._best_candidate[0], self._best_candidate[1], self._best_candidate[2], c='g')
        plt.show()

if __name__ == "__main__":
    center = Point(0.05, 0.05, 0.05)
    sphere_fit = SphereFit(use_recorded_polhemus_data = '/Ethan/2022-11-01-12-27-45.bag', plot = True)
    # sphere_fit = SphereFit()
    # sphere_fit.generate_data(1000, center, 0.08, 0.003, 0, pi, 0, pi)

    # sphere_fit.generate_data(1000, center, 0.08, 0.003, pi*1/4, pi*3/4, pi*3/8, pi*5/8)
    sphere_fit.fit_sphere([-0.1, -0.1, -0.1], [0.1, 0.1, 0.1], 0.03, 0.15)
    sphere_fit.plot_data()
