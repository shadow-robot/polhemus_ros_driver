#!/usr/bin/env python3
#
# Copyright (C) 2022 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import actionlib
from polhemus_ros_driver.msg import CalibrateAction, CalibrateGoal
import numpy as np


def feedback(feedback):
    rospy.logerr(f"Feedback... Progress: {feedback.progress}, Quality: {feedback.quality}")

def done(state, result):
    rospy.logerr(f"State: {state}, Result: {result}")

if __name__ == "__main__":
    rospy.init_node("calibration_test")

    side = "rh"
    calibration_time = 10

    client = actionlib.SimpleActionClient("/calibration_action_server", CalibrateAction)
    rospy.sleep(1)
    goal = CalibrateGoal(True, side, calibration_time)
    client.send_goal(goal, feedback_cb=feedback, done_cb=done)

