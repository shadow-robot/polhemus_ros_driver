#!/usr/bin/env python3

# Copyright (C) 2020, 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import

import rospy
import tf2_ros
import yaml
import tf
import os
from geometry_msgs.msg import TransformStamped
from polhemus_ros_driver.srv import SetStaticCalibrationTFNew
import rospkg


class StaticKnuckleBroadcaster:
    def __init__(self):
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._services = dict()
        for hand_prefix in self._get_connected_gloves():
            self._services[hand_prefix] = rospy.Service(f"/{hand_prefix}/update_static_tf_new",
                                                        SetStaticCalibrationTFNew,
                                                        self.publish_tf)

    def _get_connected_gloves(self):
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1)
        connected_glove_sides = []
        for key, value in {"polhemus_base_0": "rh", "polhemus_base_1": "lh"}.items():
            for line in tf_buffer.all_frames_as_yaml().split('\n'):
                if key in line and "parent" in line:
                    connected_glove_sides.append(value)
                    break
        rospy.logwarn(connected_glove_sides)
        return connected_glove_sides

    def publish_tf(self, req):
        success = False
        try:
            self._broadcaster.sendTransform(req.transform_stamped)
            success = True
        except Exception as e:
            rospy.logerr(e)
        return success


if __name__ == '__main__':
    rospy.init_node("sr_update_static_tf_knuckle_calibration")
    StaticKnuckleBroadcaster()
    rospy.spin()
