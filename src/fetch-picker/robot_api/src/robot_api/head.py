import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
# TODO: What messages are we going to need?
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped

""" 
This is set up for Kuri, but you can take inspiration for Fetch if you like.
"""
class Head(object):
    JOINT_PAN = 'head_pan_joint'
    JOINT_TILT = 'head_tilt_joint'
    JOINT_EYES = 'eyelids_joint'
    JOINT_HEIGHT = 0.405
    PAN_LEFT = 0.78
    PAN_NEUTRAL = 0
    PAN_RIGHT = -PAN_LEFT
    TILT_UP = -0.92
    TILT_NEUTRAL = 0.0
    TILT_DOWN = 0.29
    EYES_OPEN = 0.0
    EYES_NEUTRAL = 0.1
    EYES_CLOSED = 0.41
    EYES_HAPPY = -0.16
    EYES_SUPER_SAD = 0.15
    EYES_CLOSED_BLINK = 0.36
    # TODO: Aw shucks, ????? again?!
    # What topics should we send trajectories to for the head and eyes?
    HEAD_NS = '/head_controller/follow_joint_trajectory'
    EYES_NS = '/eyelids_controller/follow_joint_trajectory'


    def __init__(self, js, head_ns=None, eyes_ns=None):
        self._js = js
        self._head_gh = None
        self._head_goal = None
        # TODO: What is the type of these actions? 
        self._head_ac = actionlib.ActionClient(head_ns or self.HEAD_NS, FollowJointTrajectoryAction)      
        self._eyes_goal = None
        self._eyes_ac = actionlib.ActionClient(eyes_ns or self.EYES_NS, FollowJointTrajectoryAction)
        
        self._lookat_ac = actionlib.SimpleActionClient('/head_controller/point_head', PointHeadAction)
        rospy.loginfo("Waiting for look_at action server...")
        self._lookat_ac.wait_for_server()
        rospy.loginfo("Look-at action server connected.")



    def cancel(self):
        head_gh = self._head_gh
        eyes_gh = self._eyes_gh
        if head_gh:
            head_gh.cancel()
        self._head_goal = None
        self._head_gh = None
        if eyes_gh:
            eyes_gh.cancel()
        self._eyes_goal = None
        self._eyes_gh = None
        return
    
    def look_at(self, frame_id, x, y, z, duration=1.0):
        """
        Orients the head to look at a specific point in space.

        :param frame_id: The reference frame for x, y, z (e.g. 'base_link')
        :param x: X coordinate in the given frame
        :param y: Y coordinate
        :param z: Z coordinate
        :param duration: Minimum time to maintain the look direction
        """
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z

        goal.min_duration = rospy.Duration(duration)
        goal.pointing_frame = self.JOINT_PAN  # or 'head_camera_link' depending on your robot config
        goal.pointing_axis.x = 1.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 0.0

        self._lookat_ac.send_goal(goal)
        self._lookat_ac.wait_for_result()


    def eyes_to(self, radians, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's eye lids to the specified location in the duration
        specified
        
        :param radians: The eye position.  Expected to be between
        HeadClient.EYES_HAPPY and HeadClient.EYES_CLOSED
        
        :param duration: The amount of time to take to get the eyes to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
        # TODO: Build a JointTrajectoryPoint that expresses the target configuration
        # TODO: Put that point into the right container type, and target the 
        # correct joint.
        point = JointTrajectoryPoint()
        point.positions = [radians]
        point.time_from_start = rospy.Duration(duration)

        trajectory = JointTrajectory()
        trajectory.joint_names = [self.JOINT_EYES]
        trajectory.points = [point]

        return self.send_trajectory(trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def is_done(self):
        active = {
         GoalStatus.PENDING, GoalStatus.RECALLING,
         GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self._head_gh:
            if self._head_gh.get_goal_status() in active:
                return False
        if self._eyes_gh:
            if self._eyes_gh.get_goal_status() in active:
                return False
        return True

    def pan_and_tilt(self, pan, tilt, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's head to the point specified in the duration
        specified
        
        :param pan: The pan - expected to be between HeadClient.PAN_LEFT
        and HeadClient.PAN_RIGHT
        
        :param tilt: The tilt - expected to be between HeadClient.TILT_UP
        and HeadClient.TILT_DOWN
        
        :param duration: The amount of time to take to get the head to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
         # TODO: Build a JointTrajectoryPoint that expresses the target configuration
        # TODO: Put that point into the right container type, and target the 
        # correct joint.

        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(duration)

        trajectory = JointTrajectory()
        trajectory.joint_names = [self.JOINT_PAN, self.JOINT_TILT]
        trajectory.points = [point]

        return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def send_trajectory(self, traj, feedback_cb=None, done_cb=None):
        """
        Sends the specified trajectories to the head and eye controllers
        
        :param traj: A trajectory_msgs.msg.JointTrajectory.  joint_names
        are expected to match HeadClient.JOINT_PAN, JOINT_TILT and JOINT_EYES
        
        :param feedback_cb: A callable that takes one parameter - the feedback
        
        :param done_cb: A callable that takes two parameters - the goal status
        the goal handle result
        """
        for point in traj.points:
            for k in ('velocities', 'accelerations', 'effort'):
                if getattr(point, k) is None:
                    setattr(point, k, [])

            if isinstance(point.time_from_start, (int, float)):
                point.time_from_start = rospy.Duration(point.time_from_start)

        # TODO: What should be the type of the goal?
        goal = FollowJointTrajectoryGoal(trajectory=traj)

        def _handle_transition(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if done_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                if gh.get_comm_state() == CommState.DONE:
                    done_cb(gh.get_goal_status(), gh.get_result())
            return

        def _handle_feedback(gh, feedback):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if feedback_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                feedback_cb(feedback)
            return

        if self.JOINT_EYES in traj.joint_names:
            if not self._eyes_ac:
                return False
            self._eyes_goal = goal
            # TODO: How do we actually send the goal?
            self._eyes_gh = self._eyes_ac.send_goal(goal, _handle_transition, _handle_feedback)
        else:
            if not self._head_ac:
                return False
            self._head_goal = goal
            # TODO: How do we actually send the goal?
            self._head_gh = self._head_ac.send_goal(goal, _handle_transition, _handle_feedback)
        return True

    def shutdown(self):
        self.cancel()
        self._head_ac = None
        self._eyes_ac = None
        return

    def wait_for_server(self, timeout=rospy.Duration(0.0)):
        return self._head_ac.wait_for_server(timeout) and self._eyes_ac.wait_for_server(timeout)

    def wait_for_done(self, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if self.is_done():
                return True
            rate.sleep()

        return False
