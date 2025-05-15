#! /usr/bin/env python

import rospy
import tf

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('ee_pose_demo')
    wait_for_time()

    # Create a TF listener
    listener = tf.TransformListener()

    # Immediately trying to use the listener might fail because it hasn't
    # received any TF messages yet. Let's demonstrate this by trying without waiting.
    try:
        # Try to get the transform immediately (likely to fail)
        (trans, rot) = listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
        rospy.loginfo("Immediate lookup succeeded (unusual): {} {}".format(trans, rot))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo('"{}" passed to lookupTransform argument target_frame does not exist.'.format(e))

    # Now wait a bit to let the listener receive TF messages
    rospy.sleep(1.0)

    # Set up a rate to print at 1 Hz
    rate = rospy.Rate(1.0)

    # Print the pose of the wrist_roll_link at 1 Hz
    # This is typically the most useful frame for the end-effector
    while not rospy.is_shutdown():
        try:
            # Get the latest transform
            (trans, rot) = listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
            rospy.loginfo("{} {}".format(trans, rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to get transform: {}".format(e))

        # Sleep to maintain the rate
        rate.sleep()

if __name__ == '__main__':
    main()