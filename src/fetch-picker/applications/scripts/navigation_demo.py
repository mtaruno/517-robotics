#!/usr/bin/env python

import rospy
import tf.transformations as tft
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

class Navigator:
    def __init__(self):
        self.position = Point()
        self.yaw = 0
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # Control parameters
        self.angular_speed = 0.5  # radians per second
        self.linear_speed = 0.2   # meters per second
        self.angular_tolerance = 0.05  # radians
        self.position_tolerance = 0.05  # meters

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians."""
        # Convert quaternion to rotation matrix
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        # Extract yaw from rotation matrix
        return math.atan2(m[1, 0], m[0, 0])

    def odom_callback(self, msg):
        """Update position and orientation from odometry."""
        self.position = msg.pose.pose.position
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def compute_desired_yaw(self, target):
        """Compute desired yaw to face target point."""
        dx = target.x - self.position.x
        dy = target.y - self.position.y
        return math.atan2(dy, dx)

    def rotate_to(self, target_yaw):
        """Rotate to target yaw angle."""
        while not rospy.is_shutdown():
            # Compute angle difference in range [-pi, pi]
            angle_diff = target_yaw - self.yaw
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Check if we're done rotating
            if abs(angle_diff) < self.angular_tolerance:
                self.stop()
                return True

            # Create and publish rotation command
            twist = Twist()
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

    def move_forward(self, distance):
        """Move forward by specified distance."""
        start_x = self.position.x
        start_y = self.position.y
        
        while not rospy.is_shutdown():
            # Compute distance traveled
            dx = self.position.x - start_x
            dy = self.position.y - start_y
            distance_moved = math.sqrt(dx*dx + dy*dy)

            # Check if we've reached the target
            if distance_moved >= distance:
                self.stop()
                return True

            # Create and publish forward motion command
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

    def stop(self):
        """Stop the robot."""
        self.cmd_vel_pub.publish(Twist())

    def navigate_to(self, target):
        """Navigate to target point using rotate-then-move strategy."""
        # Compute desired yaw to face target
        target_yaw = self.compute_desired_yaw(target)
        rospy.loginfo(f"Rotating to yaw: {math.degrees(target_yaw)} degrees")
        
        # First rotate to face target
        self.rotate_to(target_yaw)
        
        # Compute distance to target
        dx = target.x - self.position.x
        dy = target.y - self.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"Moving forward: {distance} meters")
        
        # Then move forward
        self.move_forward(distance)

def main():
    rospy.init_node('navigation_demo')
    
    navigator = Navigator()
    rospy.sleep(1)  # Wait for odometry to start
    
    # Example: Navigate to point 1 meter ahead
    target = Point(1.0, 0.0, 0.0)
    navigator.navigate_to(target)
    
    rospy.spin()

if __name__ == '__main__':
    main()