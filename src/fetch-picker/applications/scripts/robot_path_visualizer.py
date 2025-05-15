#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
import math

class NavPath(object):
    def __init__(self):
        self._path = []
        self._last_position = None
        self._min_distance = 0.1  # Minimum distance (meters) between points
        self._marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        
    def callback(self, msg):
        current_position = msg.pose.pose.position
        
        # If this is our first point, or if we've moved far enough, add the point
        if (self._last_position is None or 
            self._get_distance(current_position, self._last_position) > self._min_distance):
            
            self._path.append(current_position)
            self._last_position = current_position
            self._publish_path()
    
    def _get_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + 
                        (p1.y - p2.y)**2 + 
                        (p1.z - p2.z)**2)
    
    def _publish_path(self):
        # Create the marker message
        marker = Marker(
            type=Marker.LINE_STRIP,
            id=1,
            lifetime=rospy.Duration(0),  # 0 = forever
            pose=Pose(),  # Use default pose (identity)
            scale=Vector3(0.05, 0.05, 0.05),  # Line width of 5cm
            header=Header(frame_id='odom'),
            color=ColorRGBA(1.0, 0.0, 0.0, 0.8),  # Red color
            points=self._path
        )
        
        # Alternative visualization using spheres
        # marker = Marker(
        #     type=Marker.SPHERE_LIST,
        #     id=1,
        #     lifetime=rospy.Duration(0),
        #     pose=Pose(),  # Use default pose (identity)
        #     scale=Vector3(0.1, 0.1, 0.1),  # Sphere diameter of 10cm
        #     header=Header(frame_id='odom'),
        #     color=ColorRGBA(0.0, 0.0, 1.0, 0.8),  # Blue color
        #     points=self._path
        # )
        
        self._marker_publisher.publish(marker)

def main():
    rospy.init_node('robot_path_visualizer')
    
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    
    rospy.loginfo("Robot path visualizer is running...")
    rospy.spin()

if __name__ == '__main__':
    main()
