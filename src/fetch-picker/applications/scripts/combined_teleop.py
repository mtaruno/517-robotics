#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from robot_api.arm import Arm
from robot_api.gripper import Gripper
from robot_teleop import GripperTeleop
from target_teleop import AutoPickTeleop

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('combined_teleop')
    wait_for_time()
    
    # Create arm and gripper objects
    arm = Arm()
    gripper = Gripper()
    
    # Create interactive marker servers
    gripper_im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    
    # Create teleop interfaces
    teleop = GripperTeleop(arm, gripper, gripper_im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    
    # Start the interfaces
    teleop.start()
    auto_pick.start()
    
    rospy.loginfo("Combined teleop interface started")
    rospy.spin()

if __name__ == '__main__':
    main()
