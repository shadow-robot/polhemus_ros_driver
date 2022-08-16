#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import, division
import rospy
import rostopic
import tf2_ros
import actionlib
import tf
import numpy as np
import math
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from polhemus_ros_driver.msg import CalibrateFeedback, CalibrateAction, CalibrateFeedback, CalibrateResult
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from enum import Enum


def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def calculate_distance(point1, point2):
    point1 = [point1.x, point1.y, point1.z]
    point2 = [point2.x, point2.y, point2.z]
    return np.linalg.norm(np.array(point1)-np.array(point2))


def sphere_fit(data):
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    A = np.zeros((len(x), 4))
    A[:, 0] = x*2
    A[:, 1] = y*2
    A[:, 2] = z*2
    A[:, 3] = 1

    f = np.zeros((len(x), 1))
    f[:, 0] = (x*x) + (y*y) + (z*z)
    C, residules, _, _ = np.linalg.lstsq(A, f, rcond=None)
    try:
        inside = (C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3]
        radius = math.sqrt(inside)
    except Exception as e:
        rospy.logwarn(f"{e} {inside}")
        radius = 10
    return radius, C[0:3], residules


class Color(Enum):
    RED = ColorRGBA(1, 0, 0, 1)
    YELLOW = ColorRGBA(1, 1, 0, 1)
    BLUE = ColorRGBA(0, 0, 1, 1)
    GREEN = ColorRGBA(0, 1, 0, 1)


COLORS = [Color.RED, Color.YELLOW, Color.BLUE, Color.GREEN]
fingers = ('ff', 'mf', 'rf', 'lf')
polhemus_to_side_prefix = {"polhemus_base_0": "rh", "polhemus_base_1": "lh"}
CENTER_MARKER_SIZE_RATIO = 0.2
CENTER_MARKER_SCALE = 0.01
CALIBRATION_FREQUENCY = 100


class DataMarker(Marker):

    _id = 0

    def __init__(self, frame_id, point, color, size=0.002):
        super().__init__()
        self.header.frame_id = frame_id
        self.header.stamp = rospy.Time.now()
        self.type = self.POINTS
        #  Markers need to have unique ids. With the line below it's ensured that
        #  every new instance has a unique, incremental id
        self.id = DataMarker._id = DataMarker._id + 1
        self.frame_locked = False
        self.points = [point]
        self.scale = Vector3(size, size, size)
        self.color = color


class SrGloveCalibration():

    _QUALITY_GOOD = 0.003
    _QUALITY_BAD = 0.01
    _ACCEPTABLE_KNUCKLE_DISTANCE = (0.015, 0.025)  # in meters

    def __init__(self):
        self._listener = tf.TransformListener()
        self._hand_side = rospy.get_param("~hand_side", 'rh')
        if self._hand_side != "lh":
            self._hand_side = "rh"
        self._index = 0 if self._hand_side == 'rh' else 1
        self._base = f"polhemus_base_{self._index}"

        self._finger_data = dict()
        connected_prefixes = self._get_connected_glove_prefixes()

        if connected_prefixes:
            self._pub = dict()
            for prefix in connected_prefixes:
                self._finger_data[prefix] = dict()
                self._pub[prefix] = rospy.Publisher(f"/data_point_marker_{prefix}", Marker, queue_size=1000)

            self._marker_server = InteractiveMarkerServer("knuckle_position_markers")
            self._action_server = actionlib.SimpleActionServer("/calibration_action_server", CalibrateAction,
                                                               execute_cb=self._calibration, auto_start=False)
            self._action_server.start()
        else:
            rospy.logerr("Not polhemus bases detected")

    def _get_connected_glove_prefixes(self):
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1)
        connected_glove_sides = []
        for key, value in polhemus_to_side_prefix.items():
            for line in tf_buffer.all_frames_as_yaml().split('\n'):
                if key in line and "parent" in line:
                    connected_glove_sides.append(value)
                    break
        return connected_glove_sides

    def _initialize_finger_data(self):
        for i, finger in enumerate(fingers):
            if not self._marker_server.get(f"{self._hand_side}_{finger}_knuckle_glove"):
                self._finger_data[self._hand_side][finger] = dict()
                #  We are tracking stations 1,2,3,4 on right hand and stations 9,10,11,12 on left hand.
                station_name = f"polhemus_station_{i + 8*self._index + 1}"
                self._finger_data[self._hand_side][finger]['polhemus_tf_name'] = station_name
                self._finger_data[self._hand_side][finger]['length'] = []
                self._finger_data[self._hand_side][finger]['residual'] = 0
                self._finger_data[self._hand_side][finger]['data'] = []
                self._finger_data[self._hand_side][finger]['center'] = self._create_marker(finger, COLORS[i].value)
                rospy.logwarn(f"Created marker { self._finger_data[self._hand_side][finger]['center'].name }")
                self._marker_server.insert(self._finger_data[self._hand_side][finger]['center'])

    def _create_marker(self, finger, color):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._base
        int_marker.name = int_marker.description = f"{self._hand_side}_{finger}_knuckle_glove"
        int_marker.scale = CENTER_MARKER_SCALE

        size_ratio = CENTER_MARKER_SIZE_RATIO
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = int_marker.scale * size_ratio
        marker.scale.y = int_marker.scale * size_ratio
        marker.scale.z = int_marker.scale * size_ratio
        marker.color = color

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

        int_marker.controls.append(self._create_control(Quaternion(0.707, 0, 0, 0.707), "rotate_x"))
        int_marker.controls.append(self._create_control(Quaternion(0.707, 0, 0, 0.707), "move_x"))
        int_marker.controls.append(self._create_control(Quaternion(0, 0, 0.707, 0.707), "rotate_z"))
        int_marker.controls.append(self._create_control(Quaternion(0, 0, 0.707, 0.707), "move_z"))
        int_marker.controls.append(self._create_control(Quaternion(0, 0.707, 0, 0.707), "rotate_y"))
        int_marker.controls.append(self._create_control(Quaternion(0, 0.707, 0, 0.707), "move_y"))

        return int_marker

    def _create_control(self, quaternion, name):
        control = InteractiveMarkerControl()
        control.orientation.w = quaternion.w
        control.orientation.x = quaternion.x
        control.orientation.y = quaternion.y
        control.orientation.z = quaternion.z
        control.name = name
        if "rotate" in name:
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        elif "move" in name:
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        return control

    def _calibration(self, goal):
        self._hand_side = goal.hand_side
        self._index = 0 if self._hand_side == 'rh' else 1
        self._base = f"polhemus_base_{self._index}"
        self._marker_server.clear()
        self._initialize_finger_data()
        self._reset_data()
        self._remove_all_markers()
        rospy.loginfo("Starting calibration..")

        rate = rospy.Rate(CALIBRATION_FREQUENCY)
        start = rospy.Time.now().to_sec()
        _feedback = CalibrateFeedback()
        _result = CalibrateResult()

        while rospy.Time.now().to_sec() - start < goal.time:
            for color_index, finger in enumerate(fingers):
                try:
                    polhemus_tf_name = self._finger_data[self._hand_side][finger]['polhemus_tf_name']
                    self._listener.waitForTransform(self._base, polhemus_tf_name, rospy.Time(), rospy.Duration(0.1))
                    pos, _ = self._listener.lookupTransform(self._base, polhemus_tf_name,
                                                            rospy.Time(0))
                    self._finger_data[self._hand_side][finger]['data'].append(pos)
                    data_point_marker = DataMarker(self._base, Point(pos[0], pos[1], pos[2]),
                                                   COLORS[color_index].value)
                    self._pub[self._hand_side].publish(data_point_marker)
                    rate.sleep()
                except Exception as error:
                    rospy.logerr(error)

            if self._action_server.is_preempt_requested():
                rospy.loginfo("Calbration stopped.")
                self._action_server.set_preempted()
                _result.success = False
                break

            _feedback.progress = ((rospy.Time.now().to_sec() - start)) / goal.time
            if len(self._finger_data[self._hand_side][finger]['data']) % 25 == 0:
                self._get_knuckle_positions(self._hand_side)
                _feedback.quality = self.get_calibration_quality()
            self._action_server.publish_feedback(_feedback)

        if not self._action_server.is_preempt_requested():
            _result.success = True
            self._action_server.set_succeeded(_result)

        rospy.loginfo("Finished calibration.")

    def _reset_data(self):
        for finger in fingers:
            self._finger_data[self._hand_side][finger]['data'] = []
            self._finger_data[self._hand_side][finger]['length'] = []

    def _remove_all_markers(self):
        marker = Marker()
        marker.header.frame_id = self._base
        marker.action = marker.DELETEALL
        self._pub[self._hand_side].publish(marker)

    def _get_knuckle_positions(self, hand_side):
        for color_index, finger in enumerate(fingers):
            solution_marker = DataMarker(self._base, self._finger_data[hand_side][finger]['center'].pose.position,
                                         COLORS[color_index].value)
            self._pub[self._hand_side].publish(solution_marker)

            r, center, residual = sphere_fit(np.array(self._finger_data[hand_side][finger]['data']))

            self._finger_data[hand_side][finger]['residual'] = residual
            self._finger_data[hand_side][finger]['length'].append(np.around(r, 4))

            center = np.around(center, 3)
            pose = Pose()
            pose.position = Point(center[0], center[1], center[2])
            pose.orientation = Quaternion(0, 0, 0, 1)
            self._marker_server.setPose(self._finger_data[hand_side][finger]['center'].name, pose)
        self._marker_server.applyChanges()

    def get_calibration_quality(self):
        quality_list = []
        for finger in fingers:
            quality = min(self._QUALITY_BAD, max(self._QUALITY_GOOD,
                          np.round(np.std(self._finger_data[self._hand_side][finger]['length']), 4)))
            quality = map_range(quality, self._QUALITY_GOOD, self._QUALITY_BAD, 100, 0)
            quality_list.append(quality)
        for distance in self.get_distances_between_knuckles():
            if not (self._ACCEPTABLE_KNUCKLE_DISTANCE[0] < distance < self._ACCEPTABLE_KNUCKLE_DISTANCE[1]):
                quality_list = [self._QUALITY_BAD, self._QUALITY_BAD, self._QUALITY_BAD, self._QUALITY_BAD]
                break
        return quality_list

    def get_distances_between_knuckles(self):
        distances = []
        for i in range(0, len(fingers)-1):
            point_1 = self._finger_data[self._hand_side][fingers[i]]['center'].pose.position
            point_2 = self._finger_data[self._hand_side][fingers[i+1]]['center'].pose.position
            distances.append(calculate_distance(point_1, point_2))
        return distances


if __name__ == "__main__":
    rospy.init_node('sr_knuckle_calibration')
    calib = SrGloveCalibration()
