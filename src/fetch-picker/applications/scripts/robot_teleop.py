#!/usr/bin/env python

import copy
import rospy
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
import robot_api
from robot_api.arm import Arm
from robot_api.gripper import Gripper

# Mesh URIs for the gripper
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

# Colors
RED = ColorRGBA(1.0, 0.0, 0.0, 0.6)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.6)
BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.6)
TRANSPARENT = ColorRGBA(0.0, 0.0, 0.0, 0.0)

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def make_6dof_controls():
    """Create 6 DOF controls for an interactive marker."""
    controls = []

    # Create X, Y, Z translation controls
    for axis, color in [('x', RED), ('y', GREEN), ('z', BLUE)]:
        control = InteractiveMarkerControl()
        control.name = f'move_{axis}'
        control.orientation.w = 1
        if axis == 'x':
            control.orientation.x = 1
        elif axis == 'y':
            control.orientation.y = 1
        elif axis == 'z':
            control.orientation.z = 1
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.always_visible = True
        controls.append(copy.deepcopy(control))

    # Create X, Y, Z rotation controls
    for axis, color in [('x', RED), ('y', GREEN), ('z', BLUE)]:
        control = InteractiveMarkerControl()
        control.name = f'rotate_{axis}'
        control.orientation.w = 1
        if axis == 'x':
            control.orientation.x = 1
        elif axis == 'y':
            control.orientation.y = 1
        elif axis == 'z':
            control.orientation.z = 1
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.always_visible = True
        controls.append(copy.deepcopy(control))

    return controls

def create_gripper_marker(color=GREEN):
    """Create markers for the gripper visualization."""
    markers = []

    # Gripper base marker
    gripper_marker = Marker()
    gripper_marker.type = Marker.MESH_RESOURCE
    gripper_marker.mesh_resource = GRIPPER_MESH
    gripper_marker.pose.position.x = 0.166  # Offset from wrist_roll_link to gripper_link
    gripper_marker.scale.x = 1.0
    gripper_marker.scale.y = 1.0
    gripper_marker.scale.z = 1.0
    gripper_marker.color = copy.deepcopy(color)
    markers.append(gripper_marker)

    # Left finger marker
    l_finger_marker = Marker()
    l_finger_marker.type = Marker.MESH_RESOURCE
    l_finger_marker.mesh_resource = L_FINGER_MESH
    l_finger_marker.pose.position.x = 0.166 + 0.15  # Offset from wrist_roll_link to finger
    l_finger_marker.pose.position.y = 0.015  # Offset in y direction
    l_finger_marker.scale.x = 1.0
    l_finger_marker.scale.y = 1.0
    l_finger_marker.scale.z = 1.0
    l_finger_marker.color = copy.deepcopy(color)
    markers.append(l_finger_marker)

    # Right finger marker
    r_finger_marker = Marker()
    r_finger_marker.type = Marker.MESH_RESOURCE
    r_finger_marker.mesh_resource = R_FINGER_MESH
    r_finger_marker.pose.position.x = 0.166 + 0.15  # Offset from wrist_roll_link to finger
    r_finger_marker.pose.position.y = -0.015  # Offset in y direction
    r_finger_marker.scale.x = 1.0
    r_finger_marker.scale.y = 1.0
    r_finger_marker.scale.z = 1.0
    r_finger_marker.color = copy.deepcopy(color)
    markers.append(r_finger_marker)

    return markers

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()

        # Add menu items
        self._menu_go_to_id = self._menu_handler.insert("Go to pose", callback=self.menu_go_to_callback)
        self._menu_open_id = self._menu_handler.insert("Open gripper", callback=self.menu_open_callback)
        self._menu_close_id = self._menu_handler.insert("Close gripper", callback=self.menu_close_callback)

        # Initialize TF listener
        self._tf_listener = tf.TransformListener()

        # Current marker pose
        self._current_pose = None

    def menu_go_to_callback(self, feedback):
        self.go_to_pose()

    def menu_open_callback(self, feedback):
        self.open_gripper()

    def menu_close_callback(self, feedback):
        self.close_gripper()

    def start(self):
        # Create interactive marker for gripper
        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "gripper_teleop"
        gripper_im.description = "Gripper Teleop"
        gripper_im.scale = 0.2  # Scale down the controls

        # Initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "base_link"
        initial_pose.pose.position.x = 0.5
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.8
        initial_pose.pose.orientation.w = 1.0

        gripper_im.pose = initial_pose.pose
        self._current_pose = initial_pose

        # Add gripper mesh markers
        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_control.always_visible = True

        # Add gripper markers to the control
        gripper_markers = create_gripper_marker()
        for marker in gripper_markers:
            gripper_control.markers.append(marker)

        gripper_im.controls.append(gripper_control)

        # Add 6DOF controls
        controls = make_6dof_controls()
        gripper_im.controls.extend(controls)

        # Add the interactive marker to the server
        self._im_server.insert(gripper_im, self.handle_feedback)
        self._menu_handler.apply(self._im_server, gripper_im.name)
        self._im_server.applyChanges()

        # Check initial pose reachability
        self.check_pose_reachability(initial_pose)

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update current pose
            pose = PoseStamped()
            pose.header = feedback.header
            pose.pose = feedback.pose
            self._current_pose = pose

            # Check if the new pose is reachable
            self.check_pose_reachability(pose)

        # Menu selection is now handled by the menu callbacks

    def check_pose_reachability(self, pose):
        # Check if the pose is reachable using IK
        is_reachable = self._arm.compute_ik(pose)

        # Update marker color based on reachability
        color = GREEN if is_reachable else RED

        # Get the marker from the server
        marker = self._im_server.get("gripper_teleop")

        # Update the color of all gripper markers
        for control in marker.controls:
            if control.interaction_mode == InteractiveMarkerControl.MENU:
                for i in range(len(control.markers)):
                    control.markers[i].color = copy.deepcopy(color)

        # Update the marker in the server
        self._im_server.insert(marker)
        self._im_server.applyChanges()

    def go_to_pose(self):
        if self._current_pose is not None:
            rospy.loginfo(f"Moving arm to pose: {self._current_pose.pose.position}")
            error = self._arm.move_to_pose(self._current_pose)
            if error is not None:
                rospy.logerr(f"Failed to move to pose: {error}")
            else:
                rospy.loginfo("Successfully moved to pose")

    def open_gripper(self):
        rospy.loginfo("Opening gripper")
        self._gripper.open()

    def close_gripper(self):
        rospy.loginfo("Closing gripper")
        self._gripper.close()

def main():
    rospy.init_node('robot_teleop')
    wait_for_time()

    # Create arm and gripper objects
    arm = Arm()
    gripper = Gripper()

    # Create interactive marker servers
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)

    # Create gripper teleop interface
    teleop = GripperTeleop(arm, gripper, im_server)
    teleop.start()

    rospy.loginfo("Robot teleop interface started")
    rospy.spin()

if __name__ == '__main__':
    main()
