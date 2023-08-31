#!/usr/bin/env python3
#
#  Copyright (C) 2022-23 Shadow Robot Company Ltd <software@shadowrobot.com>
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

from math import cos, floor, sin

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=unused-import
# Axes3D is used, indirectly, when plotting in 3D
import numpy as np
import rosbag
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from scipy.optimize import least_squares


class SphereFit:
    ''' A class for fitting a sphere to a set of points.'''
    def __init__(self, rosbag_path: str = None, data: "list[Point|list[float]]" = None, plot: bool = False) -> None:
        '''Initialise thhe sphere fitter.

        Args:
            rosbag_path: Optional, the path to the rosbag containing the data to fit the sphere to.
            data: Optional, the data to fit the sphere to.
            plot: Optional, whether to plot the data and the fitted sphere. Defaults to false.'''
        self._raw_data: "list[Point]" = []
        self._center = None
        self._best_candidate = None
        self._residuals = None
        self._plot = plot
        self.setup_plot()
        if data:
            self.set_data(data)
        elif rosbag_path:
            self.set_data_from_rosbag(rosbag_path)

    def set_data_from_rosbag(self, rosbag_path: str) -> None:
        ''' Sets the data to be used for sphere fitting from a rosbag.

        Args:
            rosbag_path: The path to the rosbag containing the data to fit the sphere to.'''
        self._raw_data = []
        local_tf_buffer = tf2_ros.Buffer()
        local_tf_buffer.clear()
        local_buffer_has_been_updated = False
        finger_polhemus_station_map = {'ff': 'polhemus_station_1', 'mf': 'polhemus_station_2',
                                       'rf': 'polhemus_station_3', 'lf': 'polhemus_station_4'}
        with rosbag.Bag(rosbag_path, 'r') as bag_file:
            for _, msg, _ in bag_file.read_messages(topics=['/tf']):
                for individual_transform in msg.transforms:
                    if individual_transform.child_frame_id in ['polhemus_station_1', 'polhemus_station_2',
                                                               'polhemus_station_3', 'polhemus_station_4']:
                        local_tf_buffer.set_transform(individual_transform, "default_authority")
                        local_buffer_has_been_updated = True

                # For each TF message, add points related to 1 finger - choose between ff, mf, rf, or lf
                if local_buffer_has_been_updated:
                    finger_transform = local_tf_buffer.lookup_transform(
                        'polhemus_base_0', finger_polhemus_station_map['ff'], rospy.Time(0))
                    point = Point()
                    point.x = finger_transform.transform.translation.x
                    point.y = finger_transform.transform.translation.y
                    point.z = finger_transform.transform.translation.z
                    self._raw_data.append(point)
                    local_buffer_has_been_updated = False

    def set_data(self, data: "list[Point|list[float]]") -> None:
        ''' Sets the data to be used for sphere fitting.

        Args:
            data: The data to fit the sphere to.'''
        self._raw_data = []
        for point in data:
            if isinstance(point, Point):
                self._raw_data.append(point)
            elif isinstance(point, list):
                self._raw_data.append(Point(point[0], point[1], point[2]))
            else:
                raise TypeError("Data must be a list of Points or lists of floats.")

    def fit_sphere(self, min_coords: "list[float]", max_coords: "list[float]", min_radius: float, max_radius: float,
                   initial_guess: "list[float]" = None, loss_function_name: str = "cauchy",
                   f_scale: float = 0.001) -> "tuple[float, list[float], list[float]]":
        ''' Fits a sphere to the data provided in the constructor, using Scipy's least squares optimization algorithm.
        See https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html for more details,
        including the available loss functions and the effects of f_scale.

        Args:
            min_coords: The minimum coordinates of the center of the sphere.
            max_coords: The maximum coordinates of the center of the sphere.
            min_radius: The minimum radius of the sphere.
            max_radius: The maximum radius of the sphere.
            initial_guess: The initial guess for the sphere fitting algorithm, in the form [x, y, z, r]. Defaults to
                [0, 0, 0, 0.08].
            loss_function_name: The name of the loss function to use. Defaults to "cauchy".
            f_scale: The value of soft margin between inlier and outlier residuals. Defaults to 0.001.

        Returns:
            A tuple containing the radius (float), center (list[float]), and residuals (list[float]) of the best
            fitting sphere.'''
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0.08]
        result = least_squares(self.sphere_errors_optimizable, initial_guess, loss=loss_function_name,
                               bounds=(min_coords + [min_radius], max_coords + [max_radius]), f_scale=f_scale)
        self._best_candidate = result.x
        self._residuals = result.fun
        self.plot_data()
        # Return radius, center, and residuals
        return self._best_candidate[3], self._best_candidate[0:3], self._residuals

    @staticmethod
    def grid_vote(data, min_coords: "list[float]", max_coords: "list[float]", min_radius: float, max_radius: float,
                  grid_points: int) -> "list[float]":
        ''' Unused prototype of an iterative grid search for a sphere matching the given data. '''
        x_candidates = np.linspace(start=min_coords[0], stop=max_coords[0], num=grid_points, endpoint=True)
        y_candidates = np.linspace(start=min_coords[1], stop=max_coords[1], num=grid_points, endpoint=True)
        z_candidates = np.linspace(start=min_coords[2], stop=max_coords[2], num=grid_points, endpoint=True)
        r_candidates = np.linspace(start=min_radius, stop=max_radius, num=grid_points, endpoint=True)
        r_error_threshold = abs(max_radius - min_radius) / (2 * (grid_points - 1))
        best_hypothesis: "list[float]" = None
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
    def point_distance(point_1: Point, point_2: Point) -> float:
        ''' Calculates the distance between two cartesian points (represented by geometry_msgs/Point objects).

        Args:
            point_1: The first point.
            point_2: The second point.'''
        return np.linalg.norm([point_1.x - point_2.x, point_1.y - point_2.y, point_1.z - point_2.z]).astype(float)

    @staticmethod
    def sphere_errors(data: "list[Point]", center_coords: Point, radius: float) -> "list[float]":
        ''' Calculates the distance between each point in data and the sphere defined by center_coords and radius.

        Args:
            data: A list of points.
            center_coords: The center of the sphere.
            radius: The radius of the sphere.'''
        return [abs(radius - SphereFit.point_distance(center_coords, point)) for point in data]

    def sphere_errors_optimizable(self, params: "list[float]") -> "list[float]":
        ''' Wrapper for sphere_errors to make it compatible with scipy's least_squares function.

        Args:
            params: A list containing the center coordinates and radius of the sphere (in the form [x, y, z, r]).'''
        return SphereFit.sphere_errors(self._raw_data, Point(params[0], params[1], params[2]), params[3])

    def setup_plot(self) -> None:
        ''' Sets up the optional plot for the sphere fitting results. '''
        if not self._plot:
            return
        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(projection='3d')
        self._ax.set_xlim3d(-0.1, 0.1)
        self._ax.set_ylim3d(0.0, 0.2)
        self._ax.set_zlim3d(-0.1, 0.1)
        self._ax.set_xlabel('X')
        self._ax.set_ylabel('Y')
        self._ax.set_zlabel('Z')

    def generate_data(self, n_points: int, center_coords: Point, radius: float, radius_std: float, polar_min: float,
                      polar_max: float, azimuth_min: float, azimuth_max: float,
                      sparse_noise_ratio: float = 0.0) -> "list[Point]":
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
        self._center = center_coords
        n_noise = floor(n_points * sparse_noise_ratio)
        n_sphere = n_points - n_noise
        radii = np.random.normal(radius, radius_std, size=n_sphere)
        polar = np.random.uniform(polar_min, polar_max, size=n_sphere)
        azimuth = np.random.uniform(azimuth_min, azimuth_max, size=n_sphere)
        result: "list[Point]" = []
        for i in range(n_sphere):
            result.append(SphereFit.point_from_polar(center_coords, radii[i], polar[i], azimuth[i]))
        if n_noise:
            for i in range(n_noise):
                result.append(SphereFit.random_cartesian(center_coords, [-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]))
        return result

    @staticmethod
    def point_from_polar(center_coords: Point, radius: float, polar: float, azimuth: float) -> Point:
        ''' Generates a point on a sphere at the given polar and azimuth angles. '''
        point = Point()
        point.x = center_coords.x + radius * sin(polar) * cos(azimuth)
        point.y = center_coords.y + radius * sin(polar) * sin(azimuth)
        point.z = center_coords.z + radius * cos(polar)
        return point

    @staticmethod
    def random_cartesian(center_point: Point, min_coords: "list[float]", max_coords: "list[float]") -> Point:
        ''' Generates a random point within the given cartesian bounds around a central point. '''
        return Point(
            center_point.x + np.random.uniform(min_coords[0], max_coords[0]),
            center_point.y + np.random.uniform(min_coords[1], max_coords[1]),
            center_point.z + np.random.uniform(min_coords[2], max_coords[2]))

    def plot_data(self, points: "list[Point]" = None) -> None:
        ''' Plots the given points (or the raw data if no points are given) and the sphere center if it exists.'''
        if not self._plot:
            return
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
