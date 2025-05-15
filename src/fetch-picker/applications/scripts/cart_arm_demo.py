#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
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
    rospy.init_node('cart_arm_demo')
    wait_for_time()

    # Create arm instance
    arm = Arm()

    # Register shutdown handler for safety
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    # Define poses for the wave motion
    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))

    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1

    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2

    gripper_poses = [ps1, ps2]

    # Move the arm between the two poses repeatedly
    rate = rospy.Rate(1)  # 1 Hz
    pose_index = 0

    while not rospy.is_shutdown():
        pose = gripper_poses[pose_index]
        rospy.loginfo("Moving to pose {}".format(pose_index + 1))

        error = arm.move_to_pose(pose)
        if error is not None:
            rospy.logerr(error)
        else:
            rospy.loginfo("Successfully moved to pose {}".format(pose_index + 1))

        # Switch to the other pose for the next iteration
        pose_index = (pose_index + 1) % len(gripper_poses)
        rate.sleep()

if __name__ == '__main__':
    main()
