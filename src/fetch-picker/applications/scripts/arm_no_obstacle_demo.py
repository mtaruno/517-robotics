#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_python import PlanningSceneInterface
import sys
import os

# Add the robot_api package to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../robot_api/src'))
import robot_api
from robot_api.arm import Arm


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_no_obstacle_demo')
    wait_for_time()

    # Wait for necessary services
    rospy.loginfo("Waiting for planning_scene service...")
    rospy.wait_for_service('/get_planning_scene')
    rospy.loginfo("Planning scene service connected.")

    # Create a planning scene interface
    planning_scene = PlanningSceneInterface('base_link')

    # Create table obstacle
    planning_scene.removeCollisionObject('table')
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0
    table_z = 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                          table_x, table_y, table_z)

    # Define poses on either side of where the divider would be
    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.5
    pose1.pose.position.y = -0.3
    pose1.pose.position.z = 0.75
    pose1.pose.orientation.w = 1

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.5
    pose2.pose.position.y = 0.3
    pose2.pose.position.z = 0.75
    pose2.pose.orientation.w = 1

    # Set up the arm
    arm = Arm()

    # Register shutdown handler
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    # Move to the poses
    kwargs = {
        'allowed_planning_time': 15,
        'execution_timeout': 10,
        'num_planning_attempts': 5,
        'replan': False
    }

    rospy.loginfo("Moving to pose 1")
    error = arm.move_to_pose(pose1, **kwargs)
    if error is not None:
        rospy.logerr('Pose 1 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 1 succeeded')

    rospy.sleep(1)

    rospy.loginfo("Moving to pose 2")
    error = arm.move_to_pose(pose2, **kwargs)
    if error is not None:
        rospy.logerr('Pose 2 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 2 succeeded')

    # Clean up the planning scene
    planning_scene.removeCollisionObject('table')


if __name__ == '__main__':
    main()
