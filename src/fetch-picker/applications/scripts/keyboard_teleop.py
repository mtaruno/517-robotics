#!/usr/bin/env python

import robot_api
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    try:
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    rospy.loginfo("Starting keyboard_teleop node...")
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('fetch_teleop_key', log_level=rospy.DEBUG)
    rospy.loginfo("Node initialized")
    
    try:
        base = robot_api.Base()
        rospy.loginfo("Base controller initialized")
    except Exception as e:
        rospy.logerr("Failed to initialize base controller: %s", str(e))
        sys.exit(1)

    x = 0
    th = 0
    status = 0
    count = 0
    speed = .2
    turn = 1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    
    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key:
                rospy.logdebug("Key pressed: %s", key)
                
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    th = moveBindings[key][1]
                elif key == ' ':
                    x = 0
                    th = 0
                    control_speed = 0
                    control_turn = 0
                elif key == '\x03':  # CTRL-C
                    break

                target_speed = speed * x
                target_turn = turn * th

                if target_speed > control_speed:
                    control_speed = min(target_speed, control_speed + 0.02)
                elif target_speed < control_speed:
                    control_speed = max(target_speed, control_speed - 0.02)
                else:
                    control_speed = target_speed

                if target_turn > control_turn:
                    control_turn = min(target_turn, control_turn + 0.1)
                elif target_turn < control_turn:
                    control_turn = max(target_turn, control_turn - 0.1)
                else:
                    control_turn = target_turn

                base.move(control_speed, control_turn)
                
            time.sleep(0.1)  # Add small delay to prevent CPU hogging
            
    except Exception as e:
        rospy.logerr("Error in main loop: %s", str(e))
    finally:
        rospy.loginfo("Stopping robot")
        base.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
