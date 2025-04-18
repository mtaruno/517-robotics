#!/usr/bin/env python3

import rospy
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../robot_api'))
from robot_api.head import Head  


def print_usage():
    # NOTE: 'look_at' and 'eyes' are optional depending on platform
    print('Usage:')
    print('    rosrun applications head_demo.py look_at FRAME_ID X Y Z')
    print('    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG')
    print('    rosrun applications head_demo.py eyes ANG')
    print('Examples:')
    print('    rosrun applications head_demo.py look_at base_link 1 0 0.3')
    print('    rosrun applications head_demo.py pan_tilt 0 0.707')
    print('    rosrun applications head_demo.py eyes 0.50')


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('head_demo')
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return

    command = argv[1]
    head = Head(js=None)

    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id = argv[2]
        x, y, z = float(argv[3]), float(argv[4]), float(argv[5])
        head.look_at(frame_id, x, y, z)
        return 
    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan = float(argv[2])
        tilt = float(argv[3])
        head.pan_and_tilt(pan, tilt)

    elif command == 'eyes':
        if len(argv) < 3:
            print_usage()
            return
        angle = float(argv[2])
        head.eyes_to(angle)

    else:
        print_usage()


if __name__ == '__main__':
    main()
