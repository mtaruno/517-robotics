#!/usr/bin/env python

import copy
import rospy
import tf
import numpy as np
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
import robot_api
from robot_api.arm import Arm
from robot_api.gripper import Gripper
import tf.transformations as tft

# Mesh URIs for the gripper
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

# Colors
RED = ColorRGBA(1.0, 0.0, 0.0, 0.6)
GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.6)
BLUE = ColorRGBA(0.0, 0.0, 1.0, 0.6)
YELLOW = ColorRGBA(1.0, 1.0, 0.0, 0.6)
PURPLE = ColorRGBA(1.0, 0.0, 1.0, 0.6)
CYAN = ColorRGBA(0.0, 1.0, 1.0, 0.6)
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

def create_box_marker(color=BLUE):
    """Create a box marker for the target object."""
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color = copy.deepcopy(color)
    return marker

def pose_to_transform(pose):
    """Convert a pose to a transformation matrix."""
    trans_matrix = np.identity(4)

    # Set translation
    trans_matrix[0, 3] = pose.position.x
    trans_matrix[1, 3] = pose.position.y
    trans_matrix[2, 3] = pose.position.z

    # Set rotation
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot_matrix = tft.quaternion_matrix(quat)
    trans_matrix[0:3, 0:3] = rot_matrix[0:3, 0:3]

    return trans_matrix

def transform_to_pose(matrix):
    """Convert a transformation matrix to a pose."""
    pose = Pose()

    # Set translation
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]

    # Set rotation
    quat = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()

        # Add menu items with callbacks
        self._menu_pregrasp_id = self._menu_handler.insert("Go to pre-grasp", callback=self.menu_pregrasp_callback)
        self._menu_grasp_id = self._menu_handler.insert("Go to grasp", callback=self.menu_grasp_callback)
        self._menu_lift_id = self._menu_handler.insert("Go to lift", callback=self.menu_lift_callback)
        self._menu_open_id = self._menu_handler.insert("Open gripper", callback=self.menu_open_callback)
        self._menu_close_id = self._menu_handler.insert("Close gripper", callback=self.menu_close_callback)

    def menu_pregrasp_callback(self, feedback):
        self.go_to_pregrasp()

    def menu_grasp_callback(self, feedback):
        self.go_to_grasp()

    def menu_lift_callback(self, feedback):
        self.go_to_lift()

    def menu_open_callback(self, feedback):
        self.open_gripper()

    def menu_close_callback(self, feedback):
        self.close_gripper()

    def menu_pick_front_callback(self, feedback):
        self.pick_from_front()

    def pick_from_front(self):
        """Execute a complete pick sequence from the front of the object."""
        if self._current_obj_pose is None:
            rospy.logerr("No object pose available")
            return

        rospy.loginfo("Executing pick from front sequence")

        # 1. Open the gripper
        rospy.loginfo("Opening gripper")
        self.open_gripper()
        rospy.sleep(1.0)

        # 2. Move to pre-grasp position
        rospy.loginfo("Moving to pre-grasp position")
        self.go_to_pregrasp()
        rospy.sleep(1.0)

        # 3. Move to grasp position
        rospy.loginfo("Moving to grasp position")
        self.go_to_grasp()
        rospy.sleep(1.0)

        # 4. Close the gripper
        rospy.loginfo("Closing gripper")
        self.close_gripper()
        rospy.sleep(1.0)

        # 5. Lift the object
        rospy.loginfo("Lifting object")
        self.go_to_lift()

        rospy.loginfo("Pick from front sequence completed")

    # Initialize TF listener and other properties
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()

        # Add menu items with callbacks
        self._menu_pick_front_id = self._menu_handler.insert("Pick from front", callback=self.menu_pick_front_callback)
        self._menu_open_id = self._menu_handler.insert("Open gripper", callback=self.menu_open_callback)

        # Add submenu for individual steps
        self._submenu_id = self._menu_handler.insert("Individual steps")
        self._menu_pregrasp_id = self._menu_handler.insert("Go to pre-grasp", parent=self._submenu_id, callback=self.menu_pregrasp_callback)
        self._menu_grasp_id = self._menu_handler.insert("Go to grasp", parent=self._submenu_id, callback=self.menu_grasp_callback)
        self._menu_lift_id = self._menu_handler.insert("Go to lift", parent=self._submenu_id, callback=self.menu_lift_callback)
        self._menu_close_id = self._menu_handler.insert("Close gripper", parent=self._submenu_id, callback=self.menu_close_callback)

        # Initialize TF listener
        self._tf_listener = tf.TransformListener()

        # Current object pose
        self._current_obj_pose = None

        # Gripper poses relative to object
        self._pregrasp_in_obj = None
        self._grasp_in_obj = None
        self._lift_in_obj = None

        # Initialize these poses
        self._init_relative_poses()

    def _init_relative_poses(self):
        """Initialize the relative poses for pre-grasp, grasp, and lift."""
        # Pre-grasp pose: 10cm behind the object
        self._pregrasp_in_obj = np.identity(4)
        self._pregrasp_in_obj[0, 3] = -0.1  # 10cm behind in x direction

        # Grasp pose: At the object
        self._grasp_in_obj = np.identity(4)

        # Lift pose: 10cm above the object
        self._lift_in_obj = np.identity(4)
        self._lift_in_obj[2, 3] = 0.1  # 10cm above in z direction

    def start(self):
        # Create interactive marker for the object
        obj_im = InteractiveMarker()
        obj_im.header.frame_id = "base_link"
        obj_im.name = "target_object"
        obj_im.description = "Target Object"
        obj_im.scale = 0.2  # Scale down the controls

        # Initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "base_link"
        initial_pose.pose.position.x = 0.7
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.7
        initial_pose.pose.orientation.w = 1.0

        obj_im.pose = initial_pose.pose
        self._current_obj_pose = initial_pose

        # Add object marker
        obj_control = InteractiveMarkerControl()
        obj_control.interaction_mode = InteractiveMarkerControl.MENU
        obj_control.always_visible = True

        # Add box marker to the control
        box_marker = create_box_marker()
        obj_control.markers.append(box_marker)

        obj_im.controls.append(obj_control)

        # Add 6DOF controls
        controls = make_6dof_controls()
        obj_im.controls.extend(controls)

        # Add the interactive marker to the server
        self._im_server.insert(obj_im, self.handle_feedback)
        self._menu_handler.apply(self._im_server, obj_im.name)
        self._im_server.applyChanges()

        # Create markers for pre-grasp, grasp, and lift poses
        self._create_gripper_pose_markers(initial_pose.pose)

    def _create_gripper_pose_markers(self, obj_pose):
        """Create markers for pre-grasp, grasp, and lift poses."""
        # Convert object pose to transformation matrix
        obj_T_base = pose_to_transform(obj_pose)

        # Compute gripper poses in base frame
        pregrasp_T_base = np.dot(obj_T_base, self._pregrasp_in_obj)
        grasp_T_base = np.dot(obj_T_base, self._grasp_in_obj)
        lift_T_base = np.dot(obj_T_base, self._lift_in_obj)

        # Convert back to poses
        pregrasp_pose = transform_to_pose(pregrasp_T_base)
        grasp_pose = transform_to_pose(grasp_T_base)
        lift_pose = transform_to_pose(lift_T_base)

        # Create markers for each pose
        self._create_pose_marker("pregrasp", pregrasp_pose, YELLOW)
        self._create_pose_marker("grasp", grasp_pose, PURPLE)
        self._create_pose_marker("lift", lift_pose, CYAN)

        # Check reachability of all poses
        pregrasp_reachable = self._arm.compute_ik(self._pose_to_posestamped(pregrasp_pose))
        grasp_reachable = self._arm.compute_ik(self._pose_to_posestamped(grasp_pose))
        lift_reachable = self._arm.compute_ik(self._pose_to_posestamped(lift_pose))

        # Update object color based on reachability of all poses
        all_reachable = pregrasp_reachable and grasp_reachable and lift_reachable
        color = GREEN if all_reachable else RED

        # Update object marker color
        marker = self._im_server.get("target_object")
        for control in marker.controls:
            if control.interaction_mode == InteractiveMarkerControl.MENU:
                control.markers[0].color = copy.deepcopy(color)

        self._im_server.insert(marker)
        self._im_server.applyChanges()

    def _create_pose_marker(self, name, pose, color):
        """Create a marker for a gripper pose."""
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.name = name
        marker.pose = pose
        marker.scale = 0.1

        # Add gripper visualization
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.NONE
        control.always_visible = True

        # Add gripper markers
        gripper_markers = create_gripper_marker(color)
        for m in gripper_markers:
            control.markers.append(m)

        marker.controls.append(control)

        # Add to server
        self._im_server.insert(marker)
        self._im_server.applyChanges()

    def _pose_to_posestamped(self, pose):
        """Convert a Pose to a PoseStamped."""
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = pose
        return ps

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update current object pose
            pose = PoseStamped()
            pose.header = feedback.header
            pose.pose = feedback.pose
            self._current_obj_pose = pose

            # Update gripper pose markers
            self._create_gripper_pose_markers(feedback.pose)

        # Menu selection is now handled by the menu callbacks

    def go_to_pregrasp(self):
        if self._current_obj_pose is not None:
            obj_T_base = pose_to_transform(self._current_obj_pose.pose)
            pregrasp_T_base = np.dot(obj_T_base, self._pregrasp_in_obj)
            pregrasp_pose = transform_to_pose(pregrasp_T_base)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = pregrasp_pose

            rospy.loginfo("Moving to pre-grasp pose")
            error = self._arm.move_to_pose(pose_stamped)
            if error is not None:
                rospy.logerr(f"Failed to move to pre-grasp pose: {error}")
            else:
                rospy.loginfo("Successfully moved to pre-grasp pose")

    def go_to_grasp(self):
        if self._current_obj_pose is not None:
            obj_T_base = pose_to_transform(self._current_obj_pose.pose)
            grasp_T_base = np.dot(obj_T_base, self._grasp_in_obj)
            grasp_pose = transform_to_pose(grasp_T_base)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = grasp_pose

            rospy.loginfo("Moving to grasp pose")
            error = self._arm.move_to_pose(pose_stamped)
            if error is not None:
                rospy.logerr(f"Failed to move to grasp pose: {error}")
            else:
                rospy.loginfo("Successfully moved to grasp pose")

    def go_to_lift(self):
        if self._current_obj_pose is not None:
            obj_T_base = pose_to_transform(self._current_obj_pose.pose)
            lift_T_base = np.dot(obj_T_base, self._lift_in_obj)
            lift_pose = transform_to_pose(lift_T_base)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = lift_pose

            rospy.loginfo("Moving to lift pose")
            error = self._arm.move_to_pose(pose_stamped)
            if error is not None:
                rospy.logerr(f"Failed to move to lift pose: {error}")
            else:
                rospy.loginfo("Successfully moved to lift pose")

    def open_gripper(self):
        rospy.loginfo("Opening gripper")
        self._gripper.open()

    def close_gripper(self):
        rospy.loginfo("Closing gripper")
        self._gripper.close()

def main():
    rospy.init_node('target_teleop')
    wait_for_time()

    # Create arm and gripper objects
    arm = Arm()
    gripper = Gripper()

    # Create interactive marker server
    im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)

    # Create auto pick teleop interface
    auto_pick = AutoPickTeleop(arm, gripper, im_server)
    auto_pick.start()

    rospy.loginfo("Target teleop interface started")
    rospy.spin()

if __name__ == '__main__':
    main()
