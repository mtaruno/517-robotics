import rospy
import actionlib

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction, OrientationConstraint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped


def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.

    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg

    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    ARM_ACTION = 'arm_controller/follow_joint_trajectory'
    MOVE_GROUP_ACTION = 'move_group'
    MOVE_DURATION = 5.0  # seconds

    def __init__(self):
        # Create actionlib client for joint control
        self._client = actionlib.SimpleActionClient(self.ARM_ACTION, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm action server...")
        self._client.wait_for_server()
        rospy.loginfo("Arm action server connected.")

        # Create actionlib client for MoveIt
        self._move_group_client = actionlib.SimpleActionClient(self.MOVE_GROUP_ACTION, MoveGroupAction)
        rospy.loginfo("Waiting for MoveGroup action server...")
        self._move_group_client.wait_for_server()
        rospy.loginfo("MoveGroup action server connected.")

        # Create service proxy for computing inverse kinematics
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()

        # TODO: Set position of trajectory point
        point.positions = arm_joints.values()

        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(self.MOVE_DURATION)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        # TODO: Add joint name to list
        goal.trajectory.points = ArmJoints.names()

        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points = [point]
        # TODO: Send goal
        self._client.send_goal(goal)

        # TODO: Wait for result
        self._client.wait_for_result()

        point = JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.time_from_start = rospy.Duration(self.MOVE_DURATION)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ArmJoints.names()
        goal.trajectory.points = [point]

        self._client.send_goal(goal)
        self._client.wait_for_result()

    def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01,
                 orientation_constraint=None):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose_stamped: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            execution_timeout: float. The maximum duration to wait for
                an arm motion to execute (or for planning to fail completely),
                in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. The number of times to compute the same
                plan. The shortest path is ultimately used. For random
                planners, this can help get shorter, less weird paths.
            plan_only: bool. If True, then this method does not execute the
                plan on the robot. Useful for determining whether this is
                likely to succeed.
            replan: bool. If True, then if an execution fails (while the arm is
                moving), then come up with a new plan and execute it.
            replan_attempts: int. How many times to replan if the execution
                fails.
            tolerance: float. The goal tolerance, in meters.
            orientation_constraint: moveit_msgs/OrientationConstraint. An orientation
                constraint to enforce during planning. If None, no orientation
                constraints will be used.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        goal_builder.group_name = group_name

        # Add orientation constraint if provided
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)

        goal = goal_builder.build()

        self._move_group_client.send_goal(goal)
        success = self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))

        if not success:
            rospy.logerr("MoveIt action timed out")
            return "TIMED_OUT"

        result = self._move_group_client.get_result()
        if result is None:
            rospy.logerr("MoveIt action returned no result")
            return "NO_RESULT"

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return moveit_error_string(result.error_code.val)
        return None

    def check_pose(self,
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01,
               orientation_constraint=None):
        """Checks if a pose is reachable via motion planning.

        Args:
            pose_stamped: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The maximum duration to wait for a
                planning result, in seconds.
            group_name: string. Either 'arm' or 'arm_with_torso'.
            tolerance: float. The goal tolerance, in meters.
            orientation_constraint: moveit_msgs/OrientationConstraint. An orientation
                constraint to enforce during planning. If None, no orientation
                constraints will be used.

        Returns:
            string describing the error if an error occurred, else None.
        """
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True,
            orientation_constraint=orientation_constraint)

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True

    def cancel_all_goals(self):
        """Cancels all goals for the arm."""
        self._client.cancel_all_goals()
        self._move_group_client.cancel_all_goals()
