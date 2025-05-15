#! /usr/bin/env python

import rospy
import subprocess
import time
import signal
import sys
import os

def signal_handler(sig, frame):
    print("\nShutting down MoveIt processes...")
    sys.exit(0)

def main():
    print("Starting MoveIt components for Fetch robot...")
    
    # Launch move_group
    print("Launching move_group...")
    move_group_process = subprocess.Popen(
        ["roslaunch", "fetch_moveit_config", "move_group.launch"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    # Wait for move_group to initialize
    time.sleep(5)
    
    # Launch RViz with MoveIt configuration
    print("Launching RViz with MoveIt configuration...")
    rviz_process = subprocess.Popen(
        ["roslaunch", "fetch_moveit_config", "moveit_rviz.launch"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    print("\nMoveIt components are now running.")
    print("You can now run the arm obstacle demos in another terminal.")
    print("Press Ctrl+C to shut down MoveIt components when done.")
    
    # Register signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Keep the script running
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
