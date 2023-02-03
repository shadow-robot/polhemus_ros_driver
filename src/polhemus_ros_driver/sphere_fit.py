#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from math import cos, floor, sin, sqrt

import matplotlib.pyplot as plt
import numpy
import rosbag
import rospkg
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from scipy.optimize import least_squares

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()


class SphereFit:
    def __init__(self, min_coords: "list[float]", max_coords: "list[float]", min_radius: float, max_radius: float, f_scale=0.001, use_recorded_polhemus_data=None, data=None, plot=False) -> None:
        '''
        Args:
            min_coords: The minimum coordinates of the center of the sphere.
            max_coords: The maximum coordinates of the center of the sphere.
            min_radius: The minimum radius of the sphere.
            max_radius: The maximum radius of the sphere
            use_recorded_polhemus_data: Load data containing real polhemus data
            data: Load other data
            plot: (boolean) plot results
        '''
        self._min_coords = min_coords
        self._max_coords = max_coords
        self._min_radius = min_radius
        self._max_radius = max_radius
        self._f_scale = f_scale
        self._raw_data = []
        self._center = None
        self._radius = None
        self._best_candidate = None
        self._residuals = None
        self.finger_polhemusstation_map = {'ff': 'polhemus_station_1', 'mf': 'polhemus_station_2',
                                           'rf': 'polhemus_station_3', 'lf': 'polhemus_station_4'}
        if plot:
            self.setup_plot()

        if data:
            for point in data:
                self._raw_data.append(Point(point[0], point[1], point[2]))

        elif use_recorded_polhemus_data:
            # Open input and output rosbags
            polhemus_driver_path = rospack.get_path('polhemus_ros_driver')
            bag_file_path = f'glove_data{use_recorded_polhemus_data}'

            local_tf_buffer = tf2_ros.Buffer()
            local_tf_buffer.clear()
            local_buffer_has_been_udpdated = False

            with rosbag.Bag(f'{polhemus_driver_path}/{bag_file_path}', 'r') as bag_file:
                for _, msg, _ in bag_file.read_messages(topics=['/tf']):
                    for individual_transform in msg.transforms:
                        if individual_transform.child_frame_id in ['polhemus_station_1', 'polhemus_station_2',
                                                                   'polhemus_station_3', 'polhemus_station_4']:
                            local_tf_buffer.set_transform(individual_transform, "default_authority")
                            local_buffer_has_been_udpdated = True

                    # For each TF message, add points related to 1 finger - choose between ff, mf, rf, or lf
                    if local_buffer_has_been_udpdated:
                        finger_transform = local_tf_buffer.lookup_transform(
                            'polhemus_base_0', self.finger_polhemusstation_map['ff'], rospy.Time(0))
                        point = Point()
                        point.x = finger_transform.transform.translation.x
                        point.y = finger_transform.transform.translation.y
                        point.z = finger_transform.transform.translation.z
                        self._raw_data.append(point)
                        local_buffer_has_been_udpdated = False

    def fit_sphere(self, initial_guess=[0, 0, 0, 0.08]):
        ''' Fits a sphere to the data provided in the constructor.

        Args:
            min_coords: 

        Returns:
            A tuple containing the radius (float), center (list[float]), and residuals (list[float]) of the best
            fitting sphere.'''
        result = least_squares(self.sphere_errors_optimizable, initial_guess, loss="cauchy",
                               bounds=(self._min_coords + [self._min_radius], self._max_coords + [self._max_radius]), f_scale=self._f_scale)
        self._best_candidate = result.x
        self._residuals = result.fun
        # Return radius, center, and residuals
        return self._best_candidate[3], self._best_candidate[0:3], self._residuals

    @staticmethod
    def grid_vote(data, min_coords, max_coords, min_radius, max_radius, grid_points):
        ''' Unused prototype of an iterative grid search for a spehere matching the given data. '''
        x_candidates = SphereFit.inclusive_arange(min_coords[0], max_coords[0], grid_points)
        y_candidates = SphereFit.inclusive_arange(min_coords[1], max_coords[1], grid_points)
        z_candidates = SphereFit.inclusive_arange(min_coords[2], max_coords[2], grid_points)
        r_candidates = SphereFit.inclusive_arange(min_radius, max_radius, grid_points)
        r_error_threshold = abs(max_radius - min_radius) / (2 * (grid_points - 1))
        best_hypothesis = None
        best_score = 0
        for x_coord in x_candidates:
            for y_coord in y_candidates:
                for z_coord in z_candidates:
                    for radius in r_candidates:
                        errors = SphereFit.sphere_errors(data, Point(x_coord, y_coord, z_coord), radius)
                        votes = sum(map(lambda error: error <= r_error_threshold, errors))
                        if votes > best_score:
                            best_score = votes
                            best_hypothesis = [x_coord, y_coord, z_coord, radius]
        return best_hypothesis

    @staticmethod
    def inclusive_arange(range_min, range_max, steps):
        arange = []
        step = (range_max - range_min) / (steps - 1)
        for i in range(steps - 1):
            arange.append(range_min + i * step)
        arange.append(range_max)
        print(arange)
        return arange

    @staticmethod
    def point_distance(point_1, point_2):
        return sqrt((point_1.x - point_2.x) ** 2 + (point_1.y - point_2.y) ** 2 + (point_1.z - point_2.z) ** 2)

    @staticmethod
    def sphere_errors(data, center_coords, radius):
        errors = []
        for point in data:
            errors.append(abs(radius - SphereFit.point_distance(center_coords, point)))
        return errors

    def sphere_errors_optimizable(self, params):
        ''' Wrapper for sphere_errors to make it compatible with scipy's least_squares function.

        Args:
            params: A list containing the center coordinates and radius of the sphere.'''
        return SphereFit.sphere_errors(self._raw_data, Point(params[0], params[1], params[2]), params[3])

    def setup_plot(self):
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(projection='3d')
        self._ax.set_xlim3d(-0.1, 0.1)
        self._ax.set_ylim3d(0.0, 0.2)
        self._ax.set_zlim3d(-0.1, 0.1)
        self._ax.set_xlabel('X')
        self._ax.set_ylabel('Y')
        self._ax.set_zlabel('Z')

    def generate_data(self, n_points, center_coords, radius, radius_std, polar_min, polar_max, azimuth_min, azimuth_max,
                      sparse_noise_ratio=0.0):
        ''' Generates N random noisy points on/near a sphere with the given parameters.

        Args:
            N: The number of points to generate.
            center_coords: The center of the sphere.
            radius: The radius of the sphere.
            radius_std: The standard deviation of the radius.
            polar_min: The minimum polar angle.
            polar_max: The maximum polar angle.
            azimuth_min: The minimum azimuth angle.
            azimuth_max: The maximum azimuth angle.
            sparse_noise_ratio: How many of the points are sparse noise (not correlated to the sphere).

            Returns:
                A list of points on/near the sphere.'''
        self._radius = radius
        self._center = center_coords
        n_noise = floor(n_points * sparse_noise_ratio)
        n_sphere = n_points - n_noise
        radii = numpy.random.normal(radius, radius_std, size=n_sphere)
        polar = numpy.random.uniform(polar_min, polar_max, size=n_sphere)
        azimuth = numpy.random.uniform(azimuth_min, azimuth_max, size=n_sphere)
        self._raw_data = []
        for i in range(n_sphere):
            self._raw_data.append(SphereFit.point_from_polar(center_coords, radii[i], polar[i], azimuth[i]))
        if n_noise:
            for i in range(n_noise):
                self._raw_data.append(SphereFit.random_cartesian(center_coords, [-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]))
        return self._raw_data

    @staticmethod
    def point_from_polar(center_coords, radius, polar, azimuth):
        point = Point()
        point.x = center_coords.x + radius * sin(polar) * cos(azimuth)
        point.y = center_coords.y + radius * sin(polar) * sin(azimuth)
        point.z = center_coords.z + radius * cos(polar)
        return point

    @staticmethod
    def random_cartesian(center_coords, min_coords, max_coords):
        return Point(
            center_coords.x + numpy.random.uniform(min_coords[0], max_coords[0]),
            center_coords.y + numpy.random.uniform(min_coords[1], max_coords[1]),
            center_coords.z + numpy.random.uniform(min_coords[2], max_coords[2]))

    def plot_data(self, points=None):
        if points is None:
            points = self._raw_data
        if self._residuals is None:
            self._ax.scatter([point.x for point in points], [point.y for point in points],
                             [point.z for point in points], marker='.')
        else:
            colour = (self._residuals - self._residuals.min()) / (self._residuals.max() - self._residuals.min())
            self._ax.scatter([point.x for point in points], [point.y for point in points],
                             [point.z for point in points], marker='.', c=colour, cmap="jet")
        if self._center is not None:
            self._ax.scatter(self._center.x, self._center.y, self._center.z, c='r', marker='.')
        if self._best_candidate is not None:
            self._ax.scatter(self._best_candidate[0], self._best_candidate[1], self._best_candidate[2], c='g')
        plt.show()


if __name__ == "__main__":
    center = Point(0.05, 0.05, 0.05)
    sphere_fit = SphereFit([-0.1, -0.1, -0.1], [0.1, 0.1, 0.1], 0.03, 0.15,  # Upper and lower bounds
                           use_recorded_polhemus_data='/Ethan/2022-11-01-12-27-45.bag',
                           plot=True)
    sphere_fit.fit_sphere(initial_guess=[0, 0, 0, 0.08])
    sphere_fit.plot_data()
