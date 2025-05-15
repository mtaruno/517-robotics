#! /usr/bin/env python

import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def transform_to_pose(matrix):
    """Convert a transformation matrix to a Pose message."""
    pose = Pose()
    
    # Extract the translation (position) from the matrix
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    
    # Extract the rotation (orientation) from the matrix and convert to quaternion
    quaternion = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    
    return pose


def pose_to_transform(pose):
    """Convert a Pose message to a transformation matrix."""
    # Create a 4x4 identity matrix
    matrix = np.identity(4)
    
    # Set the translation part
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    
    # Set the rotation part
    quaternion = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]
    rotation_matrix = tft.quaternion_matrix(quaternion)
    matrix[0:3, 0:3] = rotation_matrix[0:3, 0:3]
    
    return matrix


def axis_marker(pose_stamped):
    """Create a marker for visualizing coordinate axes."""
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'axes'
    marker.header = pose_stamped.header
    marker.pose = pose_stamped.pose
    marker.type = visualization_msgs.msg.Marker.LINE_LIST
    marker.scale.x = 0.1

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))
    marker.points.append(Point(1, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))
    marker.points.append(Point(0, 1, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))
    marker.points.append(Point(0, 0, 1))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))

    return marker


def main():
    rospy.init_node('pregrasp_demo')
    wait_for_time()
    
    # Create a publisher for visualization markers
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    rospy.sleep(0.5)
    
    # Object pose in base_link frame
    object_pose = Pose()
    object_pose.position.x = 0.6
    object_pose.position.y = -0.1
    object_pose.position.z = 0.7
    object_pose.orientation.x = 0
    object_pose.orientation.y = 0
    object_pose.orientation.z = 0.38268343
    object_pose.orientation.w = 0.92387953
    
    # Convert object pose to transformation matrix (base_link T object)
    base_T_object = pose_to_transform(object_pose)
    
    # Create pre-grasp pose in object frame
    # Identity rotation (same orientation as object)
    # Position 10cm behind object (-0.1, 0, 0) in object frame
    object_T_pregrasp = np.identity(4)
    object_T_pregrasp[0, 3] = -0.1  # 10cm behind object in object's x-axis
    
    # Compute pre-grasp pose in base_link frame
    base_T_pregrasp = np.dot(base_T_object, object_T_pregrasp)
    
    # Convert back to pose
    pregrasp_pose = transform_to_pose(base_T_pregrasp)
    
    # Print the result
    rospy.loginfo("Pre-grasp pose in base_link frame:")
    rospy.loginfo("Position: x: %f, y: %f, z: %f", 
                 pregrasp_pose.position.x, 
                 pregrasp_pose.position.y, 
                 pregrasp_pose.position.z)
    rospy.loginfo("Orientation: x: %f, y: %f, z: %f, w: %f", 
                 pregrasp_pose.orientation.x, 
                 pregrasp_pose.orientation.y, 
                 pregrasp_pose.orientation.z, 
                 pregrasp_pose.orientation.w)
    
    # Visualize the object and pre-grasp poses
    object_ps = PoseStamped()
    object_ps.header.frame_id = 'base_link'
    object_ps.pose = object_pose
    viz_pub.publish(axis_marker(object_ps))
    
    pregrasp_ps = PoseStamped()
    pregrasp_ps.header.frame_id = 'base_link'
    pregrasp_ps.pose = pregrasp_pose
    
    # Use a different namespace for the pre-grasp marker
    pregrasp_marker = axis_marker(pregrasp_ps)
    pregrasp_marker.ns = 'pregrasp_axes'
    viz_pub.publish(pregrasp_marker)
    
    # Keep the node running to maintain the visualization
    rospy.spin()


if __name__ == '__main__':
    main()
