#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import, division
import rospy
import actionlib
import tf
import numpy as np
import math
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
from polhemus_ros_driver.msg import *
from interactive_markers.interactive_marker_server import *


class KnuckleMarker():
    def __init__(self, points, size=0.002, namespace=""):
        self.position = points
        self.color = [0, 0, 0, 0]
        self.size = size
        self.namespace = namespace

    def set_color(self, color):
        if isinstance(color, str):
            if color == 'yellow':
                self.color = [1, 1, 0, 1]
            elif color == 'red':
                self.color = [1, 0, 0, 1]
            elif color == 'blue':
                self.color = [0, 0, 1, 1]
            elif color == 'green':
                self.color = [0, 1, 0, 1]
        elif isinstance(color, list):
            self.color = color

    def get_position(self):
        return self.position


class SrGloveCalibration():
    def __init__(self):
        self._listener = tf.TransformListener()
        self._index = rospy.get_param("~base_index", 0)
        self._base = f"polhemus_base_{self._index}"
        self._fingers = ['ff', 'mf', 'rf', 'lf']
        self._finger_data = dict()
        self._pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10000)
        self._id = 0
        self._initialize_finger_data()

        self._setup_interactive_markers()
        self._as = actionlib.SimpleActionServer("shadow_glove_calibration", CalibrateAction,
                                                execute_cb=self.collect_data, auto_start=False)
        self._as.start()

    def _initialize_finger_data(self):
        for index, finger in enumerate(self._fingers):
            self._finger_data[finger] = dict()
            self._finger_data[finger]['polhemus_tf_name'] = f"polhemus_station_{index + 9*self._index + 1}"
            self._finger_data[finger]['data'] = []
            self._finger_data[finger]['center'] = None
            self._finger_data[finger]['length'] = 0

    def _create_marker(self, finger, color):

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self._base
        int_marker.name = self._finger_data[finger]['polhemus_tf_name']
        int_marker.description = f"{finger}_solution"
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

    def processFeedback(self, feedback):
        print(feedback)

    def _setup_interactive_markers(self):
        self._im_server = InteractiveMarkerServer("im_server")
        colors = ['yellow', 'red', 'blue', 'green']
        for color, finger in zip(colors, self._fingers):
            self._finger_data[finger]['center'] = self._create_marker(finger, color)
            self._im_server.insert(self._finger_data[finger]['center'], self.processFeedback)

    def _knuckle_marker_to_marker(self, marker):
        mark = Marker()
        mark.header.frame_id = self._base
        mark.header.stamp = rospy.Time.now()
        mark.type = mark.POINTS
        mark.ns = marker.namespace

        mark.id = self._id
        mark.frame_locked = False
        pose = Pose()
        pose.position.x = float(marker.position[0])
        pose.position.y = float(marker.position[1])
        pose.position.z = float(marker.position[2])
        pose.orientation.w = 1

        mark.scale.x = marker.size
        mark.scale.y = marker.size
        mark.scale.z = marker.size

        mark.color.r = marker.color[0]
        mark.color.g = marker.color[1]
        mark.color.b = marker.color[2]
        mark.color.a = marker.color[3]
        mark.points = [pose.position]

        self._id += 1

        return mark

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
        C, residules, rank, singval = np.linalg.lstsq(A, f)

        #   solve for the radius
        radius = math.sqrt((C[0]*C[0])+(C[1]*C[1])+(C[2]*C[2])+C[3])
        return radius, C[0:3], residules

    def calculate_distance(self, point1, point2):
        distance = 100
        if len(point1) == 3 and len(point2) == 3:
            x = point1[0] - point2[0]
            y = point1[1] - point2[1]
            z = point1[2] - point2[2]
            distance = math.sqrt(x*x+y*y+z*z)
        return distance

    def reset_data(self):
        for finger in self._fingers:
            self._finger_data[finger]['data'] = []
            self._finger_data[finger]['length'] = 0

    def collect_data(self, goal):
        self.reset_data()
        self._remove_all_markers()

        colors = ['yellow', 'red', 'blue', 'green']
        time = 20  # seconds
        rospy.loginfo("Starting data collection..")

        rate = rospy.Rate(100)
        start = rospy.Time.now().secs

        _feedback = CalibrateFeedback()
        _result = CalibrateResult()

        while rospy.Time.now().secs - start < time:

            for color_index, finger in enumerate(self._fingers):
                self._listener.waitForTransform(self._base, self._finger_data[finger]['polhemus_tf_name'],
                                                rospy.Time(), rospy.Duration(0.1))
                (trans, _) = self._listener.lookupTransform(self._base, self._finger_data[finger]['polhemus_tf_name'],
                                                            rospy.Time(0))
                self._finger_data[finger]['data'].append(trans)
                data_point_marker = KnuckleMarker(trans, namespace="data_point")
                data_point_marker.set_color(colors[color_index])
                self._pub.publish(self._knuckle_marker_to_marker(data_point_marker))
                rate.sleep()

            if self._as.is_preempt_requested():
                rospy.loginfo("Calbration stopped ..")
                self._as.set_preempted()
                success = False
                break

            _feedback.progress = (rospy.Time.now().secs - start)/time
            if len(self._finger_data[finger]['data']) % 25 == 0:
                self._fit_data()
                _feedback.quality = self.get_calibration_quality()
            self._as.publish_feedback(_feedback)

        if not self._as.is_preempt_requested():
            _result.success = True
            self._as.set_succeeded(_result)

        rospy.loginfo("Finshed collecting data.")

    def get_calibration_quality(self):
        '''
        Figure out a way to estimate how good the current calibration is
        '''

        return np.std([0, 1, 2])

    def _fit_data(self, color=[1, 1, 1, 1], marker_namespace=""):
        colors = ['yellow', 'red', 'blue', 'green']
        for color_index, finger in enumerate(self._fingers):

            position = self._finger_data[finger]['center'].pose.position
            position = [position.x, position.y, position.z]
            solution_marker = KnuckleMarker(position, namespace="solution")
            solution_marker.set_color(colors[color_index])
            self._pub.publish(self._knuckle_marker_to_marker(solution_marker))

            r, center, _ = self._sphere_fit(np.array(self._finger_data[finger]['data']))

            center = np.around(center, 3)
            print(finger, r, [float(center[0]), float(center[1]), float(center[2])])

            pose = Pose()
            pose.position = Point(center[0], center[1], center[2])
            self._im_server.setPose(self._finger_data[finger]['center'].name, pose)
            self._im_server.applyChanges()

    def _filter_data(self):
        threshold = 0.001
        colors = ['yellow', 'red', 'blue', 'green']
        for i, finger in enumerate(self._fingers):
            data = self._finger_data[finger]['data']
            r, cords, _ = self._sphere_fit(np.array(data))
            self._finger_data[finger]['data'] = []
            for data_set in data:
                if abs(r - self.calculate_distance(cords, data_set)) < threshold:
                    self._finger_data[finger]['data'].append(data_set)
                    self._finger_data[finger]['marker'] = KnuckleMarker(data_set, 0.003,
                                                                        namespace="filtered_data_point")
                    self._finger_data[finger]['marker'].set_color(colors[i])
                    self._pub.publish(self._knuckle_marker_to_marker(self._finger_data[finger]['marker']))

    def _remove_all_markers(self):
        marker = Marker()
        marker.header.frame_id = self._base
        marker.action = marker.DELETEALL
        self._pub.publish(marker)

    def calibrate(self):
        self.collect_data()
        self._fit_data(marker_namespace="original_solution")
        rospy.sleep(1)
        self._filter_data()
        rospy.sleep(1)
        self._fit_data([255/255, 20/255, 47/255, 1], "filtered_solution")

    def get_distances_between_knuckles(self):
        distances = []
        for i in range(0, len(self._fingers)-1):
            point_1 = self._finger_data[self._fingers[i]]['center'].get_position()
            point_2 = self._finger_data[self._fingers[i+1]]['center'].get_position()
            distances.append(self.calculate_distance(point_1, point_2))
        return distances


if __name__ == "__main__":
    rospy.init_node('glove_calibration_node')
    calib = SrGloveCalibration()
