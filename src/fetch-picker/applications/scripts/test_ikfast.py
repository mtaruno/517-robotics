#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped
import sys
import os

# Add the robot_api package to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../robot_api/src'))
from robot_api.arm import Arm

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('test_ikfast')
    wait_for_time()
    
    # Create an arm object
    arm = Arm()
    
    # Create a pose to test IK
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    
    # Use command line arguments if provided, otherwise use default values
    if len(sys.argv) >= 4:
        pose.pose.position.x = float(sys.argv[1])
        pose.pose.position.y = float(sys.argv[2])
        pose.pose.position.z = float(sys.argv[3])
    else:
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 1.0
    
    pose.pose.orientation.w = 1.0
    
    # Test IK performance
    rospy.loginfo("Testing IK for pose: x={}, y={}, z={}".format(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    
    # Time the IK computation
    start_time = time.time()
    success = arm.compute_ik(pose)
    end_time = time.time()
    
    # Report results
    duration = end_time - start_time
    rospy.loginfo("IK computation took {:.6f} seconds".format(duration))
    
    if success:
        rospy.loginfo("IK solution found successfully")
    else:
        rospy.loginfo("No IK solution found")

if __name__ == '__main__':
    main()
