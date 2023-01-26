#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

from __future__ import absolute_import, division

import math
import os
from enum import Enum

import actionlib
import dynamic_reconfigure.client
import numpy as np
import rospkg
import rospy
# import rostopic
import tf2_ros
import yaml
from geometry_msgs.msg import (Point, Pose, Quaternion, TransformStamped,
                               Vector3)
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer
from std_msgs.msg import ColorRGBA
# import tf
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl, Marker)

from polhemus_ros_driver.msg import (CalibrateAction, CalibrateFeedback,
                                     CalibrateResult)
from polhemus_ros_driver.srv import Publish, PublishRequest
from sphere_fit import SphereFit


def calculate_distance(point1, point2):
    """
        Returns the distance between two points of type geometry_msgs.msg.Point
        @param point1: First point
        @param point2: Second point
    """
    point1 = [point1.x, point1.y, point1.z]
    point2 = [point2.x, point2.y, point2.z]
    return np.linalg.norm(np.array(point1)-np.array(point2))


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

class Hand():
    def __init__(self, _hand_prefix: str):
        self._hand_prefix = _hand_prefix
        self._side_name = "left" if self._hand_prefix == "lh" else "right"
        self._polhemus_base_index = 1 if self._hand_prefix == "lh" else 0
        self._polhemus_base_name = f"polhemus_base_{self._polhemus_base_index}"
        self._finger_data = {}
        self._pub = rospy.Publisher(f"/data_point_marker_{self._hand_prefix}", Marker, queue_size=1000)
        self._dynamic_reconfigure_client = dynamic_reconfigure.client.Client(
            f"/{self._hand_prefix}_sr_fingertip_hand_teleop/", timeout=30)

class SrGloveCalibration():
    def __init__(self):
        self._hands: dict[str, Hand] = {}
        for connected_prefix in self._get_connected_glove_prefixes():
            self._hands[connected_prefix] = Hand(connected_prefix)

        
        if not self._hands.keys():
            rospy.logerr("No polhemus bases detected")
            return

        self._static_transform_broadcaster = StaticTransformBroadcaster()
        self._marker_server = InteractiveMarkerServer("knuckle_position_markers")
        for hand in self._hands.values():
            self._initialize_finger_data(hand)
        self._load_default_calibrations()
        rospy.sleep(1.0)
        for hand in self._hands.values():
            self._publish_calibration(hand, save=False)
        self._update_static_tf_service = rospy.Service(f"/sr_publish_glove_calibration",
                                                       Publish,
                                                       self._publish_calibration_cb)

        self._action_server = actionlib.SimpleActionServer("/calibration_action_server", CalibrateAction,
                                                            execute_cb=self._calibration, auto_start=False)
        self._action_server.start()

    def _publish_calibration_cb(self, publish: PublishRequest):
        if (publish.side not in self._hands.keys()):
            rospy.logerr("Requested glove calibration side not found")
            return False
        self._publish_calibration(self._hands[publish.side])
        return True

    def _load_default_calibrations(self):
        """
            Loads the default calibration from the default_{side}calibration.yaml file
        """
        self._init_calibration_store()
        for hand_prefix, hand in self._hands.items():
            default_path = os.path.join(self._calibration_dir_path, f'default_calibration_{hand._side_name}.yaml')
            self._load_calibration(hand, default_path)

    def _init_calibration_store(self):
        self._calibration_dir_path = os.path.expanduser('~/shadow_glove_calibration/user_calibration')
        # If the calibration folder doesn't exist, create it
        if not os.path.exists(self._calibration_dir_path):
            os.makedirs(self._calibration_dir_path)
        # If the default calibration symlinks don't exist, create them
        for hand in self._hands.values():
            if not os.path.exists(os.path.join(self._calibration_dir_path, f'default_calibration_{hand._side_name}.yaml')):
                default_path = os.path.join(rospkg.RosPack().get_path('sr_hand_glove'),
                                            'shadow_glove_user_calibration_defaults',
                                            f'default_calibration_{hand._side_name}.yaml')
                if not os.path.exists(default_path):
                    rospy.logerr(f"Default calibration file for {hand._side_name} hand ({default_path}) not found!")
                else:
                    os.system(f'cp {default_path} {self._calibration_dir_path}/default_calibration_{hand._side_name}.yaml')

    def _load_calibration(self, hand: Hand, path: str):
        if not os.path.exists(path):
            rospy.logerr(f"Calibration file for {hand._side_name} hand {path} not found!")
        else:
            with open(path, 'r') as f:
                calibration = yaml.load(f)
                if "mf_knuckle_to_glove_source_pose" not in calibration:
                    rospy.logerr(f"Default calibration file for {self._side_long} hand {path} is missing the mf_knuckle_to_glove_source_pose key")
                    return
                necessary_keys = ["x", "y", "z"]
                missing_keys = [key for key in necessary_keys if key not in calibration["mf_knuckle_to_glove_source_pose"]]
                if missing_keys:
                    rospy.logerr(f"Default calibration file for {self._side_long} hand {path} is missing the following keys: {missing_keys}")
                    return
                pose = Pose()
                if "x" in calibration["mf_knuckle_to_glove_source_pose"]:
                    pose.position.x = -calibration["mf_knuckle_to_glove_source_pose"]["x"]
                if "y" in calibration["mf_knuckle_to_glove_source_pose"]:
                    pose.position.y = -calibration["mf_knuckle_to_glove_source_pose"]["y"]
                if "z" in calibration["mf_knuckle_to_glove_source_pose"]:
                    pose.position.z = -calibration["mf_knuckle_to_glove_source_pose"]["z"]
                pose.orientation = Quaternion(0, 0, 0, 1)
                hand._finger_data["mf"]["center"].pose = pose
                self._marker_server.insert(hand._finger_data["mf"]["center"])
                self._marker_server.applyChanges()
                if "finger_lengths" in calibration:
                    for finger in fingers:
                        if finger in calibration["finger_lengths"]:
                            hand._finger_data[finger]['length'] = [calibration["finger_lengths"][finger]]
            rospy.loginfo(f"Loaded calibration for {hand._side_name} hand from {path}")

    def _save_calibration(self, hand: Hand, path: str = None):
        if path is None:
            path = os.path.join(self._calibration_dir_path, f'default_calibration_{hand._side_name}.yaml')
        to_save = {"mf_knuckle_to_glove_source_pose": {"x": -float(hand._finger_data["mf"]["center"].pose.position.x),
                                                       "y": -float(hand._finger_data["mf"]["center"].pose.position.y),
                                                       "z": -float(hand._finger_data["mf"]["center"].pose.position.z)}}
        if any([hand._finger_data[finger]['length'] for finger in fingers]):
            to_save["finger_lengths"] = {}
            for finger in fingers:
                if hand._finger_data[finger]['length']:
                    to_save["finger_lengths"][finger] = float(hand._finger_data[finger]['length'][-1])
        with open(path, 'w') as f:
            f.seek(0)
            yaml.dump(to_save, f)
            f.truncate()
        rospy.loginfo(f"Calibration for {hand._side_name} hand saved to {path}")
        # rospy.loginfo(to_save)


    def _publish_calibration(self, hand: Hand, save: bool = True):
        """
            Publishes the calibration as a static TF
        """
        # if (hand._side_name == "left"):
        #     return
        mf_knuckle_marker = self._marker_server.get(f"{hand._hand_prefix}_mf_knuckle_glove")
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = mf_knuckle_marker.name
        transform_stamped.child_frame_id = hand._polhemus_base_name
        transform_stamped.transform.translation = Vector3(-mf_knuckle_marker.pose.position.x,
                                                          -mf_knuckle_marker.pose.position.y,
                                                          -mf_knuckle_marker.pose.position.z)
        # transform_stamped.transform.rotation = Quaternion(0.5, -0.5, 0.5, 0.5)
        # transform_stamped.transform.rotation = Quaternion(1, 0, 0, 0)
        transform_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
        # rospy.loginfo(transform_stamped)
        self._static_transform_broadcaster.sendTransform(transform_stamped)
        rospy.loginfo(f"Published glove calibration TF for {hand._side_name} hand.")
        # Updating hand dynamic reconfigure server
        new_fingertip_teleop_config = {"scaling": False}
        fingers_with_length = [finger for finger in fingers if hand._finger_data[finger]['length']]
        fingers_used_for_thumbscaling = [finger for finger in fingers_with_length if finger != "lf"]
        if fingers_with_length:
            new_fingertip_teleop_config = {"scaling": True}
            for finger in fingers_with_length:
                if hand._finger_data[finger]['length']:
                    finger_scaling = 0.096 / (hand._finger_data[finger]['length'][-1] + 0.01)
                    new_fingertip_teleop_config[finger + '_scaling_factor'] = finger_scaling
        if fingers_used_for_thumbscaling:
            new_fingertip_teleop_config['th_scaling_factor']  = sum([new_fingertip_teleop_config[finger + '_scaling_factor'] for finger in fingers_used_for_thumbscaling]) / len(fingers_used_for_thumbscaling)
        hand._dynamic_reconfigure_client.update_configuration(new_fingertip_teleop_config)
        rospy.loginfo(f"Updated fingertip teleop scaling for {hand._side_name} hand.")

        if save:
            self._save_calibration(hand)

    def _get_connected_glove_prefixes(self):
        """
            Detect connected gloves and returns the corresponding sides ['left', 'right']
        """
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(5)
        connected_glove_sides = []
        for key, value in polhemus_to_side_prefix.items():
            for line in tf_buffer.all_frames_as_yaml().split('\n'):
                if key in line and "parent" in line:
                    connected_glove_sides.append(value)
                    break
        return connected_glove_sides

    def _initialize_finger_data(self, hand: Hand):
        """
            Initializes the data per side and finger and inputs corresponding interactive markers
            into the Interactive Marker Server.
        """
        for i, finger in enumerate(fingers):
            if not self._marker_server.get(f"{hand._hand_prefix}_{finger}_knuckle_glove"):
                hand._finger_data[finger] = dict()
                #  We are tracking stations 1,2,3,4 on right hand and stations 9,10,11,12 on left hand.
                station_name = f"polhemus_station_{i + 8*hand._polhemus_base_index + 1}"
                hand._finger_data[finger]['polhemus_tf_name'] = station_name
                hand._finger_data[finger]['length'] = []
                hand._finger_data[finger]['residual'] = 0
                hand._finger_data[finger]['data'] = []
                hand._finger_data[finger]['center'] = self._create_marker(hand, finger, COLORS[i].value)
                self._marker_server.insert(hand._finger_data[finger]['center'], self._processFeedback)

    def _processFeedback(self, feedback):
        pass

    def _create_marker(self, hand: Hand, finger: str, color):
        """
            Creates an InteractiveMarker to the present the solution in Rviz.
            @param finger: Finger for which the marker gets created
            @param color: Color of the marker
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = hand._polhemus_base_name
        int_marker.name = int_marker.description = f"{hand._hand_prefix}_{finger}_knuckle_glove"
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
        """
            Creates as InteractiveMarkerControl to allow the user to drag&move the solution marker
            @param quaternion: Quaternion defining the rotations
            @param name: Name of the marker definiding allowed motion in format 'rotate/move_axis'
        """
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

    def _load_tf_callback(self, data):
        for individual_transform in data.transforms:
            for hand_prefix, hand in self._hands.items():
                for color_index, finger in enumerate(fingers):
                    if individual_transform.child_frame_id == hand._finger_data[finger]['polhemus_tf_name']:
                        pos = [individual_transform.transform.translation.x,
                            individual_transform.transform.translation.y,
                            individual_transform.transform.translation.z]

                        hand._finger_data[finger]['data'].append(pos)
                        data_point_marker = DataMarker(hand._polhemus_base_name, Point(pos[0], pos[1], pos[2]),
                                                        COLORS[color_index].value)
                        hand._pub.publish(data_point_marker)
        return


    def _calibration(self, goal):
        """
            Action server callback. This method executes the calibration procedure consisting of collecting
            TF data, fitting the data into a sphere and extracting the coordinates of the knuckles.
            @param goal: Calibration parameters of type CalibrateGoal defining the hand_side and calibration time.
        """
        if goal.hand_side not in self._hands:
            rospy.logerr(f"Hand side {goal.hand_side} is not supported")
            self._action_server.set_aborted()
            return
        hand = self._hands[goal.hand_side]
        # self._index = 0 if self._hand_side == 'rh' else 1
        # self._base = f"polhemus_base_{self._index}"
        for finger in fingers:
            self._marker_server.erase(f"{hand._hand_prefix}_{finger}_knuckle_glove")
        # self._marker_server.clear()
        self._initialize_finger_data(hand)
        self._reset_data(hand)
        self._remove_all_markers(hand)
        rospy.loginfo("Starting calibration..")

        rate = rospy.Rate(CALIBRATION_FREQUENCY)
        start = rospy.Time.now().to_sec()
        _feedback = CalibrateFeedback()
        _result = CalibrateResult()

        sub = rospy.Subscriber("/tf", TFMessage, self._load_tf_callback, queue_size=10)
        while rospy.Time.now().to_sec() - start < goal.time:
            # for color_index, finger in enumerate(fingers):
            #     try:
            #         polhemus_tf_name = self._finger_data[self._hand_side][finger]['polhemus_tf_name']
            #         self._listener.waitForTransform(self._base, polhemus_tf_name, rospy.Time(), rospy.Duration(0.1))
            #         pos, _ = self._listener.lookupTransform(self._base, polhemus_tf_name,
            #                                                 rospy.Time(0))
            #         self._finger_data[self._hand_side][finger]['data'].append(pos)
            #         data_point_marker = DataMarker(self._base, Point(pos[0], pos[1], pos[2]),
            #                                        COLORS[color_index].value)
            #         self._pub[self._hand_side].publish(data_point_marker)
            #         rate.sleep()
            #     except Exception as error:
            #         rospy.logerr(error)

            if self._action_server.is_preempt_requested():
                rospy.loginfo("Calibration stopped.")
                self._action_server.set_preempted()
                _result.success = False
                break

            _feedback.progress = ((rospy.Time.now().to_sec() - start)) / goal.time
            if math.floor(_feedback.progress * 100) % 25 == 0 and math.floor(_feedback.progress * 100) != 0: #4 times
                # print(math.floor(_feedback.progress * 100))
            # if (len(self._finger_data[self._hand_side]['ff']['data']) + 1) % 25 == 0:
                self._get_knuckle_positions(hand)
                _feedback.quality = self.get_calibration_quality(hand)
            self._action_server.publish_feedback(_feedback)

        sub.unregister()
        self._get_knuckle_positions(hand)
        # self._get_knuckle_positions(self._hand_side, plot = True)
        _feedback.quality = self.get_calibration_quality(hand)
        # print(f'Quality is {_feedback.quality}')
        if not self._action_server.is_preempt_requested():
            _result.success = True
            self._action_server.set_succeeded(_result)

        rospy.loginfo("Finished calibration.")

    def _reset_data(self, hand: Hand):
        """
            Zeroes the data.
        """
        for finger in fingers:
            hand._finger_data[finger]['data'] = []
            hand._finger_data[finger]['length'] = []

    def _remove_all_markers(self, hand: Hand):
        """
            Removes markers previously displayed in Rviz
        """
        marker = Marker()
        marker.header.frame_id = hand._polhemus_base_name
        marker.action = marker.DELETEALL
        hand._pub.publish(marker)

    def _get_knuckle_positions(self, hand: Hand, plot=False):
        """
            Updates the current solution for all fingers on the selected side.
            @param hand: Selected hand
        """
        for color_index, finger in enumerate(fingers):
            solution_marker = DataMarker(hand._polhemus_base_name, hand._finger_data[finger]['center'].pose.position,
                                         COLORS[color_index].value)
            hand._pub.publish(solution_marker)
            sphere_fit = SphereFit(data = hand._finger_data[finger]['data'], plot = plot)
            
            # print(f"Searching for centroid of finger {finger}")

            radius, center, residual = sphere_fit.fit_sphere([-0.1, -0.1, -0.1], [0.1, 0.1, 0.1], 0.03, 0.15)

            if plot:
                sphere_fit.plot_data()

            hand._finger_data[finger]['residual'] = residual
            hand._finger_data[finger]['length'].append(np.around(radius, 4))

            center = np.around(center, 3)
            pose = Pose()
            pose.position = Point(center[0], center[1], center[2])
            pose.orientation = Quaternion(0, 0, 0, 1)
            self._marker_server.setPose(hand._finger_data[finger]['center'].name, pose)
            self._marker_server.applyChanges()

    def get_calibration_quality(self, hand: Hand):
        """
            Returns the calibration quality in the form of a list. The calibration quality is measured as
            standard deviation of the residuals for each finger.
        """
        quality_list = []
        for finger in fingers:
            quality_list.append(np.std(hand._finger_data[finger]['residual']))
        return quality_list


if __name__ == "__main__":
    rospy.init_node('sr_knuckle_calibration')
    calib = SrGloveCalibration()
