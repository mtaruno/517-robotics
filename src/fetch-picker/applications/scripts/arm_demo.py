#! /usr/bin/env python

import rospy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../robot_api'))
import robot_api
from robot_api.arm import Arm
from robot_api.arm_joints import ArmJoints

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

    torso = robot_api.Torso()
    torso.set_height(robot_api.Torso.MAX_HEIGHT)

    arm = Arm()
    for vals in DISCO_POSES:
        arm.move_to_joints(ArmJoints.from_list(vals))


if __name__ == '__main__':
    main()

