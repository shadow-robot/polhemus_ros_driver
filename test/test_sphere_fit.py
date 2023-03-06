#!/usr/bin/env python3
#
#  Copyright (C) 2023 Shadow Robot Company Ltd <software@shadowrobot.com>
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

import unittest
import rostest
from geometry_msgs.msg import Point
from polhemus_ros_driver.sphere_fit import SphereFit


class TestSphereFit(unittest.TestCase):
    def test_sphere_fit(self):
        sphere_fit = SphereFit()
        center_coords = [0.01, 0.02, 0.03]
        center_point = Point(*center_coords)
        radius = 0.1
        data = sphere_fit.generate_data(
            n_points=10000, center_coords=center_point, radius=radius, radius_std=0.001, polar_min=0.0,
            polar_max=1.0, azimuth_min=0, azimuth_max=2.0, sparse_noise_ratio=0.0)
        sphere_fit.set_data(data)
        fitted_radius, fitted_center, _ = sphere_fit.fit_sphere(
            min_coords=[-0.1, -0.1, -0.1], max_coords=[0.1, 0.1, 0.1], min_radius=0.05, max_radius=0.15,
            initial_guess=[0.0, 0.0, 0.0, 0.05])
        self.assertAlmostEqual(fitted_radius, radius, places=2)
        for coord, fitted_coord in zip(center_coords, fitted_center):
            self.assertAlmostEqual(coord, fitted_coord, places=2)


if __name__ == '__main__':
    rostest.rosrun('polhemus_ros_driver', 'test_sphere_fit', TestSphereFit)
