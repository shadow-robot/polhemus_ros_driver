#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import, division
import rospy
import rostopic
import actionlib
import tf
import numpy as np
import math
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA
from polhemus_ros_driver.msg import *
from interactive_markers.interactive_marker_server import *


class DataMarker(Marker):

    _id = 0

    def __init__(self, frame_id, point, color, size=0.002):
        super().__init__()
        self.header.frame_id = frame_id
        self.header.stamp = rospy.Time.now()
        self.type = self.POINTS
        self.id = DataMarker._id = DataMarker._id + 1
        self.frame_locked = False
        self.points = [point]
        self.scale = Vector3(size, size, size)

        if isinstance(color, str):
            if color == 'yellow':
                self.color = ColorRGBA(1, 1, 0, 1)
            elif color == 'red':
                self.color = ColorRGBA(1, 0, 0, 1)
            elif color == 'blue':
                self.color = ColorRGBA(0, 0, 1, 1)
            elif color == 'green':
                self.color = ColorRGBA(0, 1, 0, 1)
        elif isinstance(color, list):
            self.color.r = color[0]
            self.color.g = color[1]
            self.color.b = color[2]
            self.color.a = color[3]


class SrGloveCalibration():
    def __init__(self):
        self._listener = tf.TransformListener()
        self._index = rospy.get_param("~base_index", 0)
        self._base = f"polhemus_base_{self._index}"
        self._fingers = ('ff', 'mf', 'rf', 'lf')
        self._finger_data = dict()
        self._im_server = None
        self._colors = ('yellow', 'red', 'blue', 'green')

        self._initialize_finger_data()
        self._pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1000)
        self._as = actionlib.SimpleActionServer("shadow_glove_calibration", CalibrateAction,
                                                execute_cb=self._calibration, auto_start=False)
        self._as.start()

    def _initialize_finger_data(self):
        self._im_server = InteractiveMarkerServer("im_server")
        for i, finger in enumerate(self._fingers):
            self._finger_data[finger] = dict()
            self._finger_data[finger]['polhemus_tf_name'] = f"polhemus_station_{i + 9*self._index + 1}"
            self._finger_data[finger]['center'] = self._create_marker(finger, self._colors[i])
            self._finger_data[finger]['length'] = 0
            self._finger_data[finger]['residual'] = 0
            self._finger_data[finger]['data'] = []

            self._im_server.insert(self._finger_data[finger]['center'], feedback_cb=self._process_feedback)

    def _create_marker(self, finger, color):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._base
        int_marker.name = int_marker.description = f"{finger}_solution"
        int_marker.scale = 0.01

        size_ratio = 0.2
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = int_marker.scale * size_ratio
        marker.scale.y = int_marker.scale * size_ratio
        marker.scale.z = int_marker.scale * size_ratio

        if isinstance(color, str):
            if color == 'yellow':
                marker.color = ColorRGBA(1, 1, 0, 1)
            elif color == 'red':
                marker.color = ColorRGBA(1, 0, 0, 1)
            elif color == 'blue':
                marker.color = ColorRGBA(0, 0, 1, 1)
            elif color == 'green':
                marker.color = ColorRGBA(0, 1, 0, 1)
        elif isinstance(color, list):
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        return int_marker

    def _process_feedback(self, feedback):
        pass

    def _calibration(self, goal):
        self._reset_data()
        self._remove_all_markers()
        rospy.loginfo("Starting calibration..")

        rate = rospy.Rate(100)
        publishing_rate = rostopic.ROSTopicHz(-1).get_hz('/tf')
        if publishing_rate:
            rate = rospy.Rate(publishing_rate)

        start = rospy.Time.now().to_sec()
        _feedback = CalibrateFeedback()
        _result = CalibrateResult()

        while rospy.Time.now().secs - start < goal.time:

            for color_index, finger in enumerate(self._fingers):
                self._listener.waitForTransform(self._base, self._finger_data[finger]['polhemus_tf_name'],
                                                rospy.Time(), rospy.Duration(0.1))
                (pos, _) = self._listener.lookupTransform(self._base, self._finger_data[finger]['polhemus_tf_name'],
                                                          rospy.Time(0))
                self._finger_data[finger]['data'].append(pos)
                data_point_marker = DataMarker(self._base, Point(pos[0], pos[1], pos[2]), self._colors[color_index])
                self._pub.publish(data_point_marker)
                rate.sleep()

            if self._as.is_preempt_requested():
                rospy.loginfo("Calbration stopped ..")
                self._as.set_preempted()
                _result.success = False
                break

            _feedback.progress = (rospy.Time.now().to_sec() - start)
            if len(self._finger_data[finger]['data']) % 50 == 0:
                self._get_knuckle_positions()
                _feedback.quality = self.get_calibration_quality()
            self._as.publish_feedback(_feedback)

        if not self._as.is_preempt_requested():
            _result.success = True
            self._as.set_succeeded(_result)

        rospy.loginfo("Finshed calibration.")

    def _reset_data(self):
        for finger in self._fingers:
            self._finger_data[finger]['data'] = []
            self._finger_data[finger]['length'] = 0

    def _remove_all_markers(self):
        marker = Marker()
        marker.header.frame_id = self._base
        marker.action = marker.DELETEALL
        self._pub.publish(marker)

    def _get_knuckle_positions(self):
        for color_index, finger in enumerate(self._fingers):
            solution_marker = DataMarker(self._base, self._finger_data[finger]['center'].pose.position,
                                         self._colors[color_index])
            self._pub.publish(solution_marker)

            r, center, residules = self._sphere_fit(np.array(self._finger_data[finger]['data'])) # [-20:]
            self._finger_data[finger]['residual'] = residules
            center = np.around(center, 3)
            #if finger == 'ff':
               #print(finger, r, float(residules),  [float(center[0]), float(center[1]), float(center[2])])

            pose = Pose()
            pose.position = Point(center[0], center[1], center[2])
            self._im_server.setPose(self._finger_data[finger]['center'].name, pose)
            self._im_server.applyChanges()

    def _sphere_fit(self, data):
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

        radius = math.sqrt((C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3])
        return radius, C[0:3], residules

    def get_calibration_quality(self):
        finger = 'ff'
        quality = []
        for finger in self._fingers:
            quality.append(self._finger_data[finger]['residual'] / len(self._finger_data[finger]['data']))
        return quality

    def get_distances_between_knuckles(self):
        distances = []
        for i in range(0, len(self._fingers)-1):
            point_1 = self._finger_data[self._fingers[i]]['center'].get_position()
            point_2 = self._finger_data[self._fingers[i+1]]['center'].get_position()
            distances.append(self.calculate_distance(point_1, point_2))
        return distances

    def calculate_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1)-np.array(point2))


if __name__ == "__main__":
    rospy.init_node('glove_calibration_node')
    calib = SrGloveCalibration()