# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy

from .arm_joints import ArmJoints
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    ARM_ACTION = 'arm_controller/follow_joint_trajectory'
    MOVE_DURATION = 5.0  # seconds

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient(self.ARM_ACTION, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm action server...")
        self._client.wait_for_server()
        rospy.loginfo("Arm action server connected.")

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory

        # TODO: Send goal
        # TODO: Wait for result
        point = JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.time_from_start = rospy.Duration(self.MOVE_DURATION)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ArmJoints.names()
        goal.trajectory.points = [point]

        self._client.send_goal(goal)
        self._client.wait_for_result()
