#!/usr/bin/env python3
#
#  Copyright (C) 2022-2023 Shadow Robot Company Ltd <software@shadowrobot.com>
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

import math
import os
from enum import Enum

from typing import List, Dict
import actionlib
from dynamic_reconfigure import client, DynamicReconfigureCallbackException, DynamicReconfigureParameterException
import numpy as np
import rospkg
import rospy
import yaml
from geometry_msgs.msg import (Point, Pose, Quaternion, TransformStamped,
                               Vector3)
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl, Marker)

from polhemus_ros_driver.msg import (CalibrateAction, CalibrateFeedback,
                                     CalibrateResult, CalibrateGoal)
from polhemus_ros_driver.srv import Publish, PublishRequest
from polhemus_ros_driver.sphere_fit import SphereFit


def calculate_distance(point1: Point, point2: Point):
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
    """ Class to store data about a marker to be published to rviz. """
    _id = 0

    def __init__(self, frame_id: str, point: Point, color: ColorRGBA, size=0.002):
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


class Hand:
    """ Class to store data about a user hand/glove; essentially a convenience struct. """
    def __init__(self, _hand_prefix: str):
        self.hand_prefix = _hand_prefix
        self.side_name = "left" if self.hand_prefix == "lh" else "right"
        self.polhemus_base_index = 1 if self.hand_prefix == "lh" else 0
        self.polhemus_base_name = f"polhemus_base_{self.polhemus_base_index}"
        self.finger_data = {}
        self.current_knuckle_tf = TransformStamped()
        self.pub = rospy.Publisher(f"/data_point_marker_{self.hand_prefix}", Marker, queue_size=1000)


class SrGloveCalibration:
    SOURCE_TO_KNUCKLE_LIMITS = [[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]]
    FINGER_LENGTH_LIMITS = [0.03, 0.15]

    def __init__(self, side: str = "right"):
        """
        Initializes the calibration action server and the interactive marker server.
        @param side: The hand(s) to calibrate - can be left, right or both
        """
        # Interactive marker server for visualizing and interacting with the calibration process
        self._marker_server = InteractiveMarkerServer("knuckle_position_markers")

        self._hands: Dict[str, Hand] = {}
        for hand_prefix in ["rh", "lh"] if side == "both" else [f"{side[0]}h"]:
            self._hands[hand_prefix] = Hand(hand_prefix)
            # Ensure the static transform publisher is ready
            rospy.sleep(1.0)
            self._initialize_finger_data(self._hands[hand_prefix])

        # Static transform broadcaster for updating the user knuckle -> polhemus base transforms
        self._static_transform_broadcaster = StaticTransformBroadcaster()
        # Ensure the static transform publisher is ready
        rospy.sleep(1.0)

        # The directory where the user calibration files are stored
        self._calibration_dir_path = os.path.expanduser('~/shadow_glove_calibration/user_calibration')

        # Load and publish the default (last) user calibration
        self._load_default_calibrations()
        for hand in self._hands.values():
            self._update_current_knuckle_tf(hand)
        self._publish_calibration(save=False)

        # Service allowing GUI to trigger publishing of the current calibration
        self._update_static_tf_service = rospy.Service("/sr_publish_glove_calibration",
                                                       Publish,
                                                       self._publish_calibration_cb)

        # Action server allowing the GUI to trigger and monitor a calibration
        self._action_server = actionlib.SimpleActionServer("/calibration_action_server", CalibrateAction,
                                                           execute_cb=self._calibration, auto_start=False)
        self._action_server.start()

    def _update_current_knuckle_tf(self, hand: Hand):
        """ Updates the current knuckle TF for a hand
            @param hand: The hand to update the TF for
        """
        rospy.loginfo(f"Updating current knuckle TF for {hand.side_name}")
        mf_knuckle_marker = self._marker_server.get(f"{hand.hand_prefix}_mf_knuckle_glove")
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = mf_knuckle_marker.name
        transform_stamped.child_frame_id = hand.polhemus_base_name
        transform_stamped.transform.translation = Vector3(-mf_knuckle_marker.pose.position.x,
                                                          -mf_knuckle_marker.pose.position.y,
                                                          -mf_knuckle_marker.pose.position.z)
        transform_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
        hand.current_knuckle_tf = transform_stamped

    def _publish_calibration_cb(self, publish: PublishRequest):
        """ Callback for the Publish calibration service
            @param publish: The request message
        """
        if publish.side not in self._hands.keys():
            rospy.logerr("Requested glove calibration side not found")
            return False
        self._update_current_knuckle_tf(self._hands[publish.side])
        self._publish_calibration()
        return True

    def _load_default_calibrations(self):
        """ Loads the default calibrations from the default_{side}calibration.yaml files """
        self._init_calibration_store()
        for hand in self._hands.values():
            default_path = os.path.join(self._calibration_dir_path, f'default_calibration_{hand.side_name}.yaml')
            self._load_calibration(hand, default_path)

    def _init_calibration_store(self):
        """ Creates the calibration folder and default calibration files if they don't exist """
        # If the calibration folder doesn't exist, create it
        if not os.path.exists(self._calibration_dir_path):
            os.makedirs(self._calibration_dir_path)
        # If the default calibration symlinks don't exist, create them
        for hand in self._hands.values():
            if not os.path.exists(os.path.join(self._calibration_dir_path,
                                               f'default_calibration_{hand.side_name}.yaml')):
                default_path = os.path.join(rospkg.RosPack().get_path('polhemus_ros_driver'),
                                            'config',
                                            'shadow_glove_user_calibration_defaults',
                                            f'default_calibration_{hand.side_name}.yaml')
                if not os.path.exists(default_path):
                    rospy.logerr(f"Default calibration file for {hand.side_name} hand ({default_path}) not found!")
                else:
                    os.system(
                        f'cp {default_path} {self._calibration_dir_path}/default_calibration_{hand.side_name}.yaml')

    def _load_calibration(self, hand: Hand, path: str):
        """ Loads a calibration file into the hand data structure
            @param hand: The hand to load the calibration into
            @param path: The path to the calibration file
        """
        if not os.path.exists(path):
            rospy.logerr(f"Calibration file for {hand.side_name} hand {path} not found!")
        else:
            with open(path, 'r', encoding='utf-8') as calibration_file:
                calibration = yaml.load(calibration_file, yaml.FullLoader)
                if "mf_knuckle_to_glove_source_pose" not in calibration:
                    rospy.logerr(f"Default calibration file for {hand.side_name} hand {path} is missing the " +
                                 "mf_knuckle_to_glove_source_pose key")
                    return
                necessary_keys = ["x", "y", "z"]
                missing_keys = [
                    key for key in necessary_keys if key not in calibration["mf_knuckle_to_glove_source_pose"]]
                if missing_keys:
                    rospy.logerr(f"Default calibration file for {hand.side_name} hand {path} is missing the following" +
                                 " keys: {missing_keys}")
                    return
                pose = Pose()
                if "x" in calibration["mf_knuckle_to_glove_source_pose"]:
                    pose.position.x = -calibration["mf_knuckle_to_glove_source_pose"]["x"]
                if "y" in calibration["mf_knuckle_to_glove_source_pose"]:
                    pose.position.y = -calibration["mf_knuckle_to_glove_source_pose"]["y"]
                if "z" in calibration["mf_knuckle_to_glove_source_pose"]:
                    pose.position.z = -calibration["mf_knuckle_to_glove_source_pose"]["z"]
                pose.orientation = Quaternion(0, 0, 0, 1)
                hand.finger_data["mf"]["center"].pose = pose
                self._marker_server.insert(hand.finger_data["mf"]["center"])
                self._marker_server.applyChanges()
                if "finger_lengths" in calibration:
                    for finger in fingers:
                        if finger in calibration["finger_lengths"]:
                            hand.finger_data[finger]['length'] = [calibration["finger_lengths"][finger]]
                else:
                    rospy.logwarn(f"Default calibration file for {hand.side_name} hand {path} is missing finger " +
                                  "lengths - scaling will be disabled.")
            rospy.loginfo(f"Loaded calibration for {hand.side_name} hand from {path}")

    def _save_calibration(self, hand: Hand, path: str = None):
        """ Saves the calibration for a hand to a file
            @param hand: The hand to save the calibration for
            @param path: The path to save the calibration to. If None, the default path will be used.
        """
        if path is None:
            path = os.path.join(self._calibration_dir_path, f'default_calibration_{hand.side_name}.yaml')
        to_save = {"mf_knuckle_to_glove_source_pose": {"x": -float(hand.finger_data["mf"]["center"].pose.position.x),
                                                       "y": -float(hand.finger_data["mf"]["center"].pose.position.y),
                                                       "z": -float(hand.finger_data["mf"]["center"].pose.position.z)}}
        if any((hand.finger_data[finger]['length'] for finger in fingers)):
            to_save["finger_lengths"] = {}
            for finger in fingers:
                if hand.finger_data[finger]['length']:
                    to_save["finger_lengths"][finger] = float(hand.finger_data[finger]['length'][-1])
        with open(path, 'w', encoding='utf-8') as calibration_file:
            calibration_file.seek(0)
            yaml.dump(to_save, calibration_file)
            calibration_file.truncate()
        rospy.loginfo(f"Calibration for {hand.side_name} hand saved to {path}")

    def _publish_calibration(self, save: bool = True):
        """ Publishes the calibration as a static TF between user knuckle and glove polhemus source
            @param save: Whether to also save the calibration to file
        """
        transform_list: List[TransformStamped] = []
        for hand in self._hands.values():
            transform_list.append(hand.current_knuckle_tf)

            # Update hand mapping node(s) parameters
            fingertip_teleop_node_name = f"/{hand.hand_prefix}_sr_fingertip_hand_teleop"
            # Build the new fingertip teleop scaling config
            # Default to scaling disabled, in the case that no fingers have a length
            new_fingertip_teleop_config = {"scaling": False}
            # Find the subset of fingers that have a length
            fingers_with_length = [finger for finger in fingers if hand.finger_data[finger]['length']]
            # Find the subset of fingers that should be used for average scaling calculation (exclude the little finger)
            fingers_used_for_thumbscaling = [finger for finger in fingers_with_length if finger != "lf"]
            # If there are fingers with lengths
            if fingers_with_length:
                # Enable scaling
                new_fingertip_teleop_config = {"scaling": True}
                # Calculate the scaling factor for each finger
                for finger in fingers_with_length:
                    if hand.finger_data[finger]['length']:
                        finger_scaling = 0.096 / (hand.finger_data[finger]['length'][-1] + 0.01)
                        new_fingertip_teleop_config[f'{finger}_scaling_factor'] = finger_scaling
            # If there are fingers with lengths that are not the little finger
            if fingers_used_for_thumbscaling:
                # Calculate the average scaling factor, to be used for the thumb
                new_fingertip_teleop_config['th_scaling_factor'] = (sum([
                    new_fingertip_teleop_config[f'{finger}_scaling_factor'] for finger in
                    fingers_used_for_thumbscaling]) / len(fingers_used_for_thumbscaling))
            # Try to update the fingertip teleop node's dynamic reconfigure server
            try:
                dynamic_reconfigure_client = client.Client(
                    f"/{fingertip_teleop_node_name}", timeout=1)
                dynamic_reconfigure_client.update_configuration(new_fingertip_teleop_config)
            except rospy.ROSException:
                rospy.loginfo(f"Could not communicate with dynamic reconfigure server at '{fingertip_teleop_node_name}'"
                              ". In case the node isn't running yet, scaling parameters have been updated on the "
                              "parameter server, and will be applied when the node is started.")
                # Fall back to updating the parameter server directly. This helps during teleop startup, when this node
                # loads the last user finger lengths from file, but fingertip teleop node dynamic reconfigure servers
                # are not yet available. The fingertip teleop launch files do load default scaling parameters, but that
                # should happen before this, so the loaded user saclings will override them and be applied when the
                # fingertip teleop dynamic reconfigure servers initialize.
                for param_name, param_value in new_fingertip_teleop_config.items():
                    rospy.set_param(f"{fingertip_teleop_node_name}/{param_name}", param_value)
            # The dynamic reconfigure server is available, but the update failed for some other reason
            except (DynamicReconfigureParameterException, DynamicReconfigureCallbackException) as err:
                rospy.logwarn(f"Could not update fingertip mapping scaling for {hand.side_name} hand: {err}")
            else:
                rospy.loginfo(f"Updated fingertip mapping scaling for {hand.side_name} hand.")

            if save:
                self._save_calibration(hand)

        try:
            self._static_transform_broadcaster.sendTransform(transform_list)
        except Exception as err:
            rospy.logerr(f"Could not publish glove calibration TF(s): {err}.")
        else:
            rospy.loginfo("Published glove calibration TF(s).")

    def _initialize_finger_data(self, hand: Hand):
        """
            Initializes the data per side and finger and inputs corresponding interactive markers
            into the Interactive Marker Server.
            @param hand: The hand to initialize the finger data for
        """
        for i, finger in enumerate(fingers):
            if not self._marker_server.get(f"{hand.hand_prefix}_{finger}_knuckle_glove"):
                hand.finger_data[finger] = {}
                #  We are tracking stations 1,2,3,4 on right hand and stations 9,10,11,12 on left hand.
                station_name = f"polhemus_station_{i + 8*hand.polhemus_base_index + 1}"
                hand.finger_data[finger]['polhemus_tf_name'] = station_name
                hand.finger_data[finger]['length'] = []
                hand.finger_data[finger]['residual'] = 0
                hand.finger_data[finger]['data'] = []
                hand.finger_data[finger]['center'] = self._create_marker(hand, finger, COLORS[i].value)
                self._marker_server.insert(hand.finger_data[finger]['center'])

    def _create_marker(self, hand: Hand, finger: str, color):
        """
            Creates an InteractiveMarker to the present the solution in Rviz.
            @param finger: Finger for which the marker gets created
            @param color: Color of the marker
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = hand.polhemus_base_name
        int_marker.name = int_marker.description = f"{hand.hand_prefix}_{finger}_knuckle_glove"
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

    @staticmethod
    def _create_control(quaternion: Quaternion, name: str):
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
        """ Callback for received TF data. """
        for individual_transform in data.transforms:
            for hand in self._hands.values():
                for color_index, finger in enumerate(fingers):
                    if individual_transform.child_frame_id == hand.finger_data[finger]['polhemus_tf_name']:
                        pos = [individual_transform.transform.translation.x,
                               individual_transform.transform.translation.y,
                               individual_transform.transform.translation.z]

                        hand.finger_data[finger]['data'].append(pos)
                        data_point_marker = DataMarker(hand.polhemus_base_name, Point(*pos),
                                                       COLORS[color_index].value)
                        hand.pub.publish(data_point_marker)

    def _calibration(self, goal: CalibrateGoal):
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
        for finger in fingers:
            self._marker_server.erase(f"{hand.hand_prefix}_{finger}_knuckle_glove")
        self._initialize_finger_data(hand)
        self._reset_data(hand)
        self._remove_all_markers(hand)
        rospy.loginfo("Starting calibration..")

        start = rospy.Time.now().to_sec()
        _feedback = CalibrateFeedback()
        _result = CalibrateResult()

        sub = rospy.Subscriber("/tf", TFMessage, self._load_tf_callback, queue_size=10)
        while rospy.Time.now().to_sec() - start < goal.time:
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Calibration stopped.")
                self._action_server.set_preempted()
                _result.success = False
                break

            _feedback.progress = ((rospy.Time.now().to_sec() - start)) / goal.time
            if math.floor(_feedback.progress * 100) % 25 == 0 and math.floor(_feedback.progress * 100) != 0:
                self._get_knuckle_positions(hand)
                _feedback.quality = self.get_calibration_quality(hand)
            self._action_server.publish_feedback(_feedback)

        sub.unregister()
        self._get_knuckle_positions(hand)
        _feedback.quality = self.get_calibration_quality(hand)
        if not self._action_server.is_preempt_requested():
            _result.success = True
            self._action_server.set_succeeded(_result)

        rospy.loginfo("Finished calibration.")

    @staticmethod
    def _reset_data(hand: Hand):
        """
            Zeroes the data.
            @param hand: Selected hand
        """
        for finger in fingers:
            hand.finger_data[finger]['data'] = []
            hand.finger_data[finger]['length'] = []

    @staticmethod
    def _remove_all_markers(hand: Hand):
        """
            Removes markers previously displayed in Rviz
            @param hand: Selected hand
        """
        marker = Marker()
        marker.header.frame_id = hand.polhemus_base_name
        marker.action = marker.DELETEALL
        hand.pub.publish(marker)

    def _get_knuckle_positions(self, hand: Hand, plot=False):
        """
            Updates the current solution for all fingers on the selected hand.
            @param hand: Selected hand
            @param plot: If True, plots the data points and the fitted sphere
        """
        for color_index, finger in enumerate(fingers):
            solution_marker = DataMarker(hand.polhemus_base_name, hand.finger_data[finger]['center'].pose.position,
                                         COLORS[color_index].value)
            hand.pub.publish(solution_marker)
            sphere_fit = SphereFit(data=hand.finger_data[finger]['data'], plot=plot)

            radius, center, residual = sphere_fit.fit_sphere(
                SrGloveCalibration.SOURCE_TO_KNUCKLE_LIMITS[0], SrGloveCalibration.SOURCE_TO_KNUCKLE_LIMITS[1],
                SrGloveCalibration.FINGER_LENGTH_LIMITS[0], SrGloveCalibration.FINGER_LENGTH_LIMITS[1])

            if plot:
                sphere_fit.plot_data()

            hand.finger_data[finger]['residual'] = residual
            hand.finger_data[finger]['length'].append(np.around(radius, 4).item())

            center = np.around(center, 3)
            pose = Pose()
            pose.position = Point(center[0], center[1], center[2])
            pose.orientation = Quaternion(0, 0, 0, 1)
            self._marker_server.setPose(hand.finger_data[finger]['center'].name, pose)
            self._marker_server.applyChanges()

    @staticmethod
    def get_calibration_quality(hand: Hand):
        """
            Returns the calibration quality in the form of a list. The calibration quality is measured as
            standard deviation of the residuals for each finger.
            @param hand: Selected hand
        """
        quality_list: List[float] = []
        for finger in fingers:
            quality_list.append(np.std(hand.finger_data[finger]['residual']))
        return quality_list


if __name__ == "__main__":
    rospy.init_node('sr_knuckle_calibration')
    hand_side = rospy.get_param("~side", "both")
    sr_glove_calibration = SrGloveCalibration(side=hand_side)
