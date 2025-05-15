#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
import sys
import os

# Add the robot_api package to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../robot_api/src'))
from robot_api.arm import Arm
from robot_api.arm_joints import ArmJoints


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def move_to_neutral(arm):
    """Move the arm to a neutral position well above the table."""
    rospy.loginfo("Moving to neutral position high above the table")
    joints = ArmJoints()
    # Use a position that's safely above the table
    joints.set_shoulder_pan(0.0)
    joints.set_shoulder_lift(-0.8)  # Negative value lifts the arm up
    joints.set_upperarm_roll(0.0)
    joints.set_elbow_flex(1.0)
    joints.set_forearm_roll(0.0)
    joints.set_wrist_flex(1.0)  # Bend wrist to keep gripper horizontal
    joints.set_wrist_roll(0.0)
    arm.move_to_joints(joints)
    rospy.sleep(3)  # Give more time to reach the position


def main():
    rospy.init_node('orientation_constraint_demo')
    wait_for_time()

    # Set up the arm
    arm = Arm()

    # Register shutdown handler
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    rospy.loginfo("Running orientation constraint demo (keeping gripper upright)")

    # Move to a neutral position first - BEFORE setting up the planning scene
    move_to_neutral(arm)

    # Now that the arm is in a safe position, set up the planning scene
    rospy.loginfo("Setting up planning scene with arm in safe position")
    planning_scene = PlanningSceneInterface('base_link')

    # First remove any existing objects to start clean
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('divider')
    planning_scene.removeAttachedObject('tray')
    rospy.sleep(1.0)  # Give time for objects to be removed

    # Create table obstacle
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0
    table_z = 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                          table_x, table_y, table_z)

    # Wait to ensure the planning scene is updated
    rospy.loginfo("Waiting for planning scene to update...")
    rospy.sleep(2.0)

    # Define poses for the demo - well above the table
    pickup_pose = PoseStamped()
    pickup_pose.header.frame_id = 'base_link'
    pickup_pose.pose.position.x = 0.7
    pickup_pose.pose.position.y = -0.3
    pickup_pose.pose.position.z = 1.1  # Much higher position above the table
    pickup_pose.pose.orientation.w = 1

    place_pose = PoseStamped()
    place_pose.header.frame_id = 'base_link'
    place_pose.pose.position.x = 0.7
    place_pose.pose.position.y = 0.3
    place_pose.pose.position.z = 1.1  # Much higher position above the table
    place_pose.pose.orientation.w = 1

    rospy.loginfo("Target poses set at height z=1.1m (well above the table at z=0.6m)")

    # Planning parameters
    kwargs = {
        'allowed_planning_time': 20,
        'execution_timeout': 20,
        'num_planning_attempts': 10,
        'replan': True,
        'tolerance': 0.05
    }

    # Create orientation constraint to keep gripper upright
    oc = OrientationConstraint()
    oc.header.frame_id = 'base_link'
    oc.link_name = 'wrist_roll_link'
    oc.orientation.w = 1.0
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14  # Allow rotation around z-axis
    oc.weight = 1.0
    rospy.loginfo("Created orientation constraint to keep gripper upright")

    # Move to pickup position with orientation constraint
    rospy.loginfo("Moving to pickup position with orientation constraint")
    pickup_kwargs = kwargs.copy()
    pickup_kwargs['orientation_constraint'] = oc

    error = arm.move_to_pose(pickup_pose, **pickup_kwargs)
    if error is not None:
        rospy.logerr('Pickup pose failed: {}'.format(error))
        return
    rospy.loginfo("Successfully reached pickup position")

    # Attach a tray to the gripper
    rospy.loginfo("Attaching tray to gripper (simulating picking up a tray)")
    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
    ]
    planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                             frame_attached_to, frames_okay_to_collide_with)
    planning_scene.setColor('tray', 1, 0, 1)  # Purple color
    planning_scene.sendColors()

    rospy.sleep(2)  # Give time for visualization

    # Move to place position with tray attached and orientation constraint
    rospy.loginfo("Moving to place position with tray")
    rospy.loginfo("Keeping the gripper upright (as if carrying a glass of water)")

    # Use more planning time for this move
    place_kwargs = kwargs.copy()
    place_kwargs['allowed_planning_time'] = 30
    place_kwargs['num_planning_attempts'] = 15
    place_kwargs['orientation_constraint'] = oc

    error = arm.move_to_pose(place_pose, **place_kwargs)
    if error is not None:
        rospy.logerr('Place pose failed: {}'.format(error))
    else:
        rospy.loginfo('Successfully moved to place position while keeping tray level!')

    # Clean up the planning scene
    planning_scene.removeAttachedObject('tray')
    planning_scene.removeCollisionObject('table')


if __name__ == '__main__':
    main()
