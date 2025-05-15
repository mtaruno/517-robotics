#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from moveit_python import PlanningSceneInterface
import sys
import os
import copy

# Add the robot_api package to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../robot_api/src'))
import robot_api
from robot_api.arm import Arm
from robot_api.arm_joints import ArmJoints
from robot_api.torso import Torso


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def test_pose_reachability(arm, pose, planning_time=5.0, group_name='arm_with_torso'):
    """Test if a pose is reachable without actually moving the arm."""
    rospy.loginfo("Testing reachability of pose: x={}, y={}, z={}".format(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))

    # Try with increased tolerance for better success rate
    error = arm.check_pose(pose,
                          allowed_planning_time=planning_time,
                          group_name=group_name,
                          tolerance=0.05)

    if error is None:
        rospy.loginfo("Pose is reachable")
        return True
    else:
        # If it fails, try again with even more tolerance before giving up
        rospy.logwarn("First reachability check failed: {}. Trying with increased tolerance...".format(error))
        error = arm.check_pose(pose,
                              allowed_planning_time=planning_time,
                              group_name=group_name,
                              tolerance=0.1)
        if error is None:
            rospy.loginfo("Pose is reachable with increased tolerance")
            return True
        else:
            rospy.logwarn("Pose is NOT reachable: {}".format(error))
            return False


def move_to_neutral(arm):
    """Move the arm to a neutral position that's safe for adding obstacles."""
    rospy.loginfo("Moving to neutral position")
    joints = ArmJoints()
    joints.set_shoulder_pan(0.0)
    joints.set_shoulder_lift(-0.5)  # Lift the shoulder up to avoid collisions
    joints.set_upperarm_roll(0.0)
    joints.set_elbow_flex(1.0)      # Bend the elbow more
    joints.set_forearm_roll(0.0)
    joints.set_wrist_flex(0.5)
    joints.set_wrist_roll(0.0)
    arm.move_to_joints(joints)
    rospy.sleep(2)  # Wait longer to ensure the arm reaches the position


def visualize_poses(pickup_pose, place_pose):
    """Publish poses as a PoseArray for visualization in RViz."""
    pose_pub = rospy.Publisher('/visualization_poses', PoseArray, queue_size=1, latch=True)

    # Create PoseArray message
    pose_array = PoseArray()
    pose_array.header.frame_id = 'base_link'
    pose_array.header.stamp = rospy.Time.now()

    # Add pickup pose
    pickup = copy.deepcopy(pickup_pose.pose)
    pose_array.poses.append(pickup)

    # Add place pose
    place = copy.deepcopy(place_pose.pose)
    pose_array.poses.append(place)

    # Publish the pose array
    pose_pub.publish(pose_array)
    rospy.loginfo("Published poses for visualization. Add a PoseArray display in RViz pointing to /visualization_poses")


def main():
    rospy.init_node('arm_obstacle_with_divider_demo')
    wait_for_time()

    # Wait for necessary services
    rospy.loginfo("Waiting for planning_scene service...")
    rospy.wait_for_service('/get_planning_scene')
    rospy.loginfo("Planning scene service connected.")

    # Create a planning scene interface
    planning_scene = PlanningSceneInterface('base_link')

    # Define poses for the demo - positioned to force going around the divider
    home_pose = PoseStamped()
    home_pose.header.frame_id = 'base_link'
    home_pose.pose.position.x = 0.65  # Further forward
    home_pose.pose.position.y = 0.0   # Centered
    home_pose.pose.position.z = 0.9   # Higher to avoid obstacles
    home_pose.pose.orientation.w = 1

    # Pickup pose - clearly on the left side of the divider
    pickup_pose = PoseStamped()
    pickup_pose.header.frame_id = 'base_link'
    pickup_pose.pose.position.x = 0.7   # Further forward
    pickup_pose.pose.position.y = -0.25 # More to the left
    pickup_pose.pose.position.z = 0.8   # Higher
    pickup_pose.pose.orientation.w = 1

    # Place pose - clearly on the right side of the divider
    place_pose = PoseStamped()
    place_pose.header.frame_id = 'base_link'
    place_pose.pose.position.x = 0.7   # Further forward
    place_pose.pose.position.y = 0.25  # More to the right
    place_pose.pose.position.z = 0.8   # Higher
    place_pose.pose.orientation.w = 1

    # Visualize the poses in RViz
    visualize_poses(pickup_pose, place_pose)

    # Set up the arm and torso
    arm = Arm()
    torso = Torso()

    # Register shutdown handler
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    # # Set torso to maximum height to help with planning
    # rospy.loginfo("Setting torso to maximum height")
    # torso.set_height(Torso.MAX_HEIGHT)
    # rospy.loginfo("Waiting for torso to reach maximum height...")
    # rospy.sleep(5)  # Increased wait time to ensure torso reaches target height

    # Move to a neutral position first
    rospy.loginfo("Moving to a safe neutral position")
    move_to_neutral(arm)
    rospy.sleep(2)  # Wait for arm to reach neutral position

    # NOW add obstacles after the arm is in a safe position
    rospy.loginfo("Adding obstacles to the planning scene...")

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

    # Create divider obstacle - ENABLED for this demo with increased size
    planning_scene.removeCollisionObject('divider')
    size_x = 0.3
    size_y = 0.05  # Increased thickness from 0.01 to 0.05
    size_z = 0.6   # Increased height from 0.4 to 0.6
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = 0
    z = table_z + (table_size_z / 2) + (size_z / 2)
    planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

    # Add a second divider to make it more substantial - positioned higher to avoid initial collision
    planning_scene.removeCollisionObject('divider_top')
    size_x_top = 0.4  # Wider at the top
    size_y_top = 0.05
    size_z_top = 0.05
    x_top = x
    y_top = 0
    z_top = z + (size_z / 2) + (size_z_top / 2) + 0.05  # Added 5cm to avoid collision with arm
    planning_scene.addBox('divider_top', size_x_top, size_y_top, size_z_top, x_top, y_top, z_top)

    # Wait to ensure the planning scene is updated
    rospy.loginfo("Waiting for planning scene to update...")
    rospy.sleep(2.0)  # Increased wait time

    # Move to the poses with more generous parameters
    kwargs = {
        'allowed_planning_time': 30,      # Increase from 15 to 30
        'execution_timeout': 35,          # Increase from 10 to 35
        'num_planning_attempts': 10,      # Increase from 5 to 10
        'replan': True,                   # Enable replanning
        'tolerance': 0.05,                # Increase tolerance from 0.01 to 0.05
        'group_name': 'arm_with_torso'    # Use arm_with_torso instead of just arm
    }

    # Run the demo with attached object
    rospy.loginfo("\n=== DEMO WITH ATTACHED OBJECT AND DIVIDER ===")

    # Display the current planning scene in RViz
    rospy.loginfo("Planning scene has been set up with table and divider")
    rospy.loginfo("Please check RViz to see the obstacles")
    rospy.sleep(2)

    # Test if pickup pose is reachable
    rospy.loginfo("Testing if pickup pose is reachable...")
    if test_pose_reachability(arm, pickup_pose, planning_time=10.0):
        # Move to pickup position
        rospy.loginfo("Moving to pickup position (left side of divider)")
        error = arm.move_to_pose(pickup_pose, **kwargs)
        if error is not None:
            rospy.logerr('Pickup pose failed: {}'.format(error))
            return
        rospy.loginfo("Successfully reached pickup position")
    else:
        rospy.logerr("Pickup pose is not reachable, skipping")
        return

    # Attach the tray to the gripper
    rospy.loginfo("Attaching tray to gripper")
    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
    ]
    planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                             frame_attached_to, frames_okay_to_collide_with)
    planning_scene.setColor('tray', 1, 0, 1)  # Purple color
    planning_scene.sendColors()

    rospy.sleep(2)  # Give more time for visualization and planning scene update

    # Test if place pose is reachable with the tray attached
    rospy.loginfo("Testing if place pose on the other side of divider is reachable...")
    if test_pose_reachability(arm, place_pose, planning_time=15.0):  # Increased planning time
        # Move to place position with tray attached
        rospy.loginfo("Moving to place position (right side of divider, with tray attached)")
        rospy.loginfo("This should force the arm to go AROUND or OVER the divider, not through it")

        # Increase planning time for this critical move
        place_kwargs = kwargs.copy()
        place_kwargs['allowed_planning_time'] = 45  # Give more time for this complex planning
        place_kwargs['num_planning_attempts'] = 15  # More attempts

        error = arm.move_to_pose(place_pose, **place_kwargs)
        if error is not None:
            rospy.logerr('Place pose with tray failed: {}'.format(error))
        else:
            rospy.loginfo('Successfully moved to place position on the other side of the divider!')
    else:
        rospy.logerr("Place pose is not reachable with tray attached, skipping")

    # Clean up the planning scene
    planning_scene.removeAttachedObject('tray')
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('divider')
    planning_scene.removeCollisionObject('divider_top')


if __name__ == '__main__':
    main()
