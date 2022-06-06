#!/usr/bin/env python3
#
# Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import
from math import pi


class SourcePoseCalibration(object):
    def __init__(self):
        self.CONST_SOURCE_ORIENTATION = [0, 0.1, -pi]
        self.CONST_SOURCE_X_OFFSET = 0.008
        self.CONST_SOURCE_Z_OFFSET = 0.016
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
    def decalibrate(self, x, y, z):
        knuckle_thickness = (z - self.CONST_SOURCE_Z_OFFSET) * 2
        knuckle_to_source = -(x + self.CONST_SOURCE_X_OFFSET)
        self.check_if_values_within_range(knuckle_thickness, knuckle_to_source)
        return [knuckle_thickness, knuckle_to_source]

    def check_if_values_within_range(self, knuckle_thickness, knuckle_to_source):
        if (not self.knuckle_thickness_accepted_range[0] <= knuckle_thickness <=
                self.knuckle_thickness_accepted_range[1]):
            raise ValueError("Knuckle thickness outside of accepted range")
        if (not self.knuckle_to_source_accepted_range[0] <= knuckle_to_source <=
                self.knuckle_to_source_accepted_range[1]):
            raise ValueError("Knuckle to source value outside of accepted range")
