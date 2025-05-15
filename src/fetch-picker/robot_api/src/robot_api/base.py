import rospy
import copy
import math
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Base(object):
    """Base controls the mobile base portion of the Fetch robot."""

    def __init__(self):
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self._latest_odom = None
        rospy.sleep(0.5)

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds."""
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._pub.publish(twist)

    def stop(self):
        """Stops the mobile base from moving."""
        twist = Twist()
        self._pub.publish(twist)
        rospy.sleep(0.1)

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # Wait for at least one odom message
        while self._latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Record start position
        start = copy.deepcopy(self._latest_odom)
        rate = rospy.Rate(10)

        # Calculate distance moved
        def get_distance_traveled():
            curr_x = self._latest_odom.pose.pose.position.x
            curr_y = self._latest_odom.pose.pose.position.y
            start_x = start.pose.pose.position.x
            start_y = start.pose.pose.position.y
            dx = curr_x - start_x
            dy = curr_y - start_y
            return math.sqrt(dx*dx + dy*dy)

        # Move until we reach distance
        while not rospy.is_shutdown():
            distance_traveled = get_distance_traveled()
            if abs(distance_traveled) >= abs(distance):
                break
            
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

        self.stop()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # Wait for at least one odom message
        while self._latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Record start position
        start = copy.deepcopy(self._latest_odom)
        rate = rospy.Rate(10)

        # Get yaw from quaternion
        def get_yaw(odom):
            q = odom.pose.pose.orientation
            angles = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return angles[2]

        # Handle angles greater than 2π or less than -2π
        angular_distance = angular_distance % (2 * math.pi)
        if angular_distance > math.pi:
            angular_distance -= 2 * math.pi
        elif angular_distance < -math.pi:
            angular_distance += 2 * math.pi

        start_yaw = get_yaw(start)
        while not rospy.is_shutdown():
            current_yaw = get_yaw(self._latest_odom)
            # Calculate angle turned so far
            angle_turned = current_yaw - start_yaw
            # Normalize angle
            angle_turned = angle_turned % (2 * math.pi)
            if angle_turned > math.pi:
                angle_turned -= 2 * math.pi
            elif angle_turned < -math.pi:
                angle_turned += 2 * math.pi

            if abs(angle_turned) >= abs(angular_distance):
                break

            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

        self.stop()

            
