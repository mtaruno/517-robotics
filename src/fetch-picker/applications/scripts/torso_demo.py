#! /usr/bin/env python

import rospy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../robot_api'))
from robot_api.torso import Torso

def print_usage():
    print('Moves the torso to a certain height between [0.0, 0.4]')
    print('Usage: rosrun applications torso_demo.py 0.4')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('torso_demo')
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return

    try:
        height = float(argv[1])
    except ValueError:
        print('Invalid height. Must be a float between 0.0 and 0.4.')
        return

    if not 0.0 <= height <= 0.4:
        print('Height must be between 0.0 and 0.4 meters.')
        return

    torso = Torso()
    rospy.loginfo(f"Moving torso to height: {height}")
    torso.set_height(height)



if __name__ == '__main__':
    main()
