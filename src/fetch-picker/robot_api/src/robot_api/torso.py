#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


# TODO: ACTION_NAME = ???
# TODO: JOINT_NAME = ???
ACTION_NAME = '/torso_controller/follow_joint_trajectory'
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso controller action server...")
        self._client.wait_for_server()
        rospy.loginfo("Torso controller action server connected.")

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        height = max(self.MIN_HEIGHT, min(self.MAX_HEIGHT, height))
        point = JointTrajectoryPoint()
        point.positions = [height]
        point.time_from_start = rospy.Duration(TIME_FROM_START)


        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [JOINT_NAME]
        goal.trajectory.points = [point]

        # TODO: Send goal
        # TODO: Wait for result
        self._client.send_goal(goal)
        self._client.wait_for_result()

        rospy.loginfo(f"Torso moved to {height:.2f}m.")
