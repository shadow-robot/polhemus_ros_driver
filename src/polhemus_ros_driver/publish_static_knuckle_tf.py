#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

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
