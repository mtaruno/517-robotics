#! /usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
from control_msgs.msg import GripperCommandGoal, GripperCommandAction

import rospy
import control_msgs.msg
import actionlib

# TODO: ACTION_NAME = ???
ACTION_NAME = '/gripper_controller/gripper_action'
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient(ACTION_NAME, GripperCommandAction)
        rospy.loginfo("Waiting for gripper action server...")
        self._client.wait_for_server()
        rospy.loginfo("Gripper action server connected.")

    def open(self):
        """Opens the gripper.
        """
        # TODO: Create goal
        # TODO: Send goal
        # TODO: Wait for result
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS
        goal.command.max_effort = -1.0 # negative effort means no force limit
        # send goal
        self._client.send_goal(goal)
        # wait for result
        self._client.wait_for_result()
        result = self._client.get_result()

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        # TODO: Create goal
        # TODO: Send goal
        # TODO: Wait for result
        goal = GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max(max_effort, self.MIN_EFFORT)
        # send goal
        self._client.send_goal(goal)
        # wait for result
        self._client.wait_for_result()