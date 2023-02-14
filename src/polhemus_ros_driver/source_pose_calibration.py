#!/usr/bin/env python3
#
#  Copyright (C) 2022 Shadow Robot Company Ltd <software@shadowrobot.com>
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

from math import pi


class SourcePoseCalibration:
    CONST_SOURCE_ORIENTATION = [0, 0.1, -pi]
    CONST_SOURCE_X_OFFSET = 0.008
    CONST_SOURCE_Z_OFFSET = 0.016

    def __init__(self):
        self.knuckle_thickness_accepted_range = [0.015, 0.045]
        self.knuckle_to_source_accepted_range = [0.01, 0.06]

    def calibrate(self, knuckle_thickness, knuckle_to_source):
        self.check_if_values_within_range(knuckle_thickness, knuckle_to_source)
        return [
            -(knuckle_to_source + self.CONST_SOURCE_X_OFFSET),
            0,
            knuckle_thickness / 2 + self.CONST_SOURCE_Z_OFFSET
        ] + self.CONST_SOURCE_ORIENTATION

    # Currently orientation does not affect decalibration
    def decalibrate(self, x_coord, _y_coord, z_coord):
        knuckle_thickness = (z_coord - self.CONST_SOURCE_Z_OFFSET) * 2
        knuckle_to_source = -(x_coord + self.CONST_SOURCE_X_OFFSET)
        self.check_if_values_within_range(knuckle_thickness, knuckle_to_source)
        return [knuckle_thickness, knuckle_to_source]

    def check_if_values_within_range(self, knuckle_thickness, knuckle_to_source):
        if (not self.knuckle_thickness_accepted_range[0] <= knuckle_thickness <=
                self.knuckle_thickness_accepted_range[1]):
            raise ValueError("Knuckle thickness outside of accepted range")
        if (not self.knuckle_to_source_accepted_range[0] <= knuckle_to_source <=
                self.knuckle_to_source_accepted_range[1]):
            raise ValueError("Knuckle to source value outside of accepted range")
