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
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from polhemus_ros_driver.msg import *


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
        self._calibration_running = True

        self._as = actionlib.SimpleActionServer("shadow_glove_calibration", CalibrateAction, 
                                                execute_cb=self.collect_data, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):

        rate = rospy.Rate(1)
        success = True

        feedback = CalibrateFeedback()
        for i in range(0,50):
            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted")
                self._as.set_preempted()
                success = False
                break
            self._as.publish_feedback(feedback)
            print(i)
            rate.sleep()

        if success:
            self._result = i == 50
            self._as.set_succeeded(self._result)
        
    def _initialize_finger_data(self):
        for index, finger in enumerate(self._fingers):
            self._finger_data[finger] = dict()
            self._finger_data[finger]['polhemus_tf_name'] = f"polhemus_station_{index + 9*self._index + 1}"
            self._finger_data[finger]['knuckle_position'] = 0
            self._finger_data[finger]['data'] = []
            self._finger_data[finger]['marker'] = None
            self._finger_data[finger]['center'] = None

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

    def collect_data(self, goal):
        colors = ['yellow', 'red', 'blue', 'green']
        time = 10  # seconds
        rospy.loginfo("Starting data collection..")

        rate = rospy.Rate(100)
        start = rospy.Time.now().secs

        while rospy.Time.now().secs - start < time and self._calibration_running:
            for color_index, finger in enumerate(self._fingers):           
                self._listener.waitForTransform(self._base, self._finger_data[finger]['polhemus_tf_name'],
                                                rospy.Time(), rospy.Duration(0.1))
                (trans, _) = self._listener.lookupTransform(self._base, self._finger_data[finger]['polhemus_tf_name'],
                                                            rospy.Time(0))
                self._finger_data[finger]['data'].append(trans)
                self._finger_data[finger]['marker'] = KnuckleMarker(trans, namespace="data_point")
                self._finger_data[finger]['marker'].set_color(colors[color_index])
                self._pub.publish(self._knuckle_marker_to_marker(self._finger_data[finger]['marker']))
                rate.sleep()

            if self._as.is_preempt_requested():
                rospy.loginfo("Calbration stopped..")
                self._as.set_preempted()
                success = False
                break

        self._result = 50 == 50
        self._as.set_succeeded(self._result)

        rospy.loginfo("Finshed collecting data.")

    def _fit_data(self, color=[1, 1, 1, 1], marker_namespace=""):
        rospy.loginfo("Fitting data..")
        for finger in self._fingers:
            r, cords, _ = self._sphere_fit(np.array(self._finger_data[finger]['data']))
            self._finger_data[finger]['marker'] = KnuckleMarker(cords, 0.003, namespace=marker_namespace)
            self._finger_data[finger]['marker'].set_color(color)
            cords = np.around(cords, 3)
            print(finger, r, [float(cords[0]), float(cords[1]), float(cords[2])])
            self._finger_data[finger]['center'] = self._finger_data[finger]['marker']
            self._pub.publish(self._knuckle_marker_to_marker(self._finger_data[finger]['center']))
        rospy.loginfo("Done!")

    def _filter_data(self):
        threshold = 0.001
        colors = ['blue', 'green', 'yellow', 'red']
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
        for i in range(0, len(self._fingers)-1):
            point_1 = self._finger_data[self._fingers[i]]['center'].get_position()
            point_2 = self._finger_data[self._fingers[i+1]]['center'].get_position()
            distance = self.calculate_distance(point_1, point_2)
            print(self._fingers[i], self._fingers[i+1], distance)


if __name__ == "__main__":
    rospy.init_node('glove_calibration_node')
    calib = SrGloveCalibration()
    #calib.calibrate()
    #calib.get_distances_between_knuckles()
    rospy.spin()
