#!/usr/bin/env python

import copy
import rospy
import tf
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Pose, Point
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

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

class GripperTeleop(object):
    def __init__(self):
        self._arm = Arm()
        self._gripper = Gripper()
        self._im_server = InteractiveMarkerServer('gripper_teleop', q_size=2)
        
        # Create menu handler
        self._menu_handler = MenuHandler()
        self._menu_go_id = self._menu_handler.insert("Go to pose")
        self._menu_open_id = self._menu_handler.insert("Open gripper")
        self._menu_close_id = self._menu_handler.insert("Close gripper")
        
        # Current pose
        self._current_pose = None
        
    def start(self):
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "gripper_teleop"
        int_marker.description = "Gripper Teleop"
        int_marker.scale = 0.2
        
        # Initial pose
        int_marker.pose.position.x = 0.5
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.8
        int_marker.pose.orientation.w = 1.0
        
        # Create gripper marker
        gripper_control = InteractiveMarkerControl()
        gripper_control.always_visible = True
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        
        # Add gripper mesh
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.position.x = 0.166  # Offset from wrist_roll_link to gripper_link
        gripper_marker.scale.x = 1.0
        gripper_marker.scale.y = 1.0
        gripper_marker.scale.z = 1.0
        gripper_marker.color = GREEN
        gripper_control.markers.append(gripper_marker)
        
        # Add left finger mesh
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = L_FINGER_MESH
        l_finger_marker.pose.position.x = 0.166 + 0.15
        l_finger_marker.pose.position.y = 0.015
        l_finger_marker.scale.x = 1.0
        l_finger_marker.scale.y = 1.0
        l_finger_marker.scale.z = 1.0
        l_finger_marker.color = GREEN
        gripper_control.markers.append(l_finger_marker)
        
        # Add right finger mesh
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = R_FINGER_MESH
        r_finger_marker.pose.position.x = 0.166 + 0.15
        r_finger_marker.pose.position.y = -0.015
        r_finger_marker.scale.x = 1.0
        r_finger_marker.scale.y = 1.0
        r_finger_marker.scale.z = 1.0
        r_finger_marker.color = GREEN
        gripper_control.markers.append(r_finger_marker)
        
        int_marker.controls.append(gripper_control)
        
        # Add 6-DOF controls
        self._add_6dof_controls(int_marker)
        
        # Add the marker to the server
        self._im_server.insert(int_marker, self._process_feedback)
        self._menu_handler.apply(self._im_server, int_marker.name)
        self._im_server.applyChanges()
        
        # Store initial pose
        self._current_pose = self._create_pose_stamped(int_marker.pose)
        
        # Check initial reachability
        self._check_reachability(self._current_pose)
        
    def _add_6dof_controls(self, int_marker):
        # X axis control
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # Y axis control
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # Z axis control
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
    
    def _create_pose_stamped(self, pose):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = pose
        return ps
    
    def _check_reachability(self, pose_stamped):
        # Check if pose is reachable
        is_reachable = self._arm.compute_ik(pose_stamped)
        
        # Update marker color
        color = GREEN if is_reachable else RED
        
        # Get the marker
        int_marker = self._im_server.get("gripper_teleop")
        
        # Update colors
        for control in int_marker.controls:
            if control.interaction_mode == InteractiveMarkerControl.MENU:
                for marker in control.markers:
                    marker.color = copy.deepcopy(color)
        
        # Update the marker
        self._im_server.insert(int_marker)
        self._im_server.applyChanges()
    
    def _process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update current pose
            self._current_pose = self._create_pose_stamped(feedback.pose)
            
            # Check reachability
            self._check_reachability(self._current_pose)
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == self._menu_go_id:
                self._go_to_pose()
            elif feedback.menu_entry_id == self._menu_open_id:
                self._open_gripper()
            elif feedback.menu_entry_id == self._menu_close_id:
                self._close_gripper()
    
    def _go_to_pose(self):
        if self._current_pose is not None:
            rospy.loginfo("Moving to pose: {}".format(self._current_pose.pose.position))
            error = self._arm.move_to_pose(self._current_pose)
            if error is not None:
                rospy.logerr("Failed to move to pose: {}".format(error))
            else:
                rospy.loginfo("Successfully moved to pose")
    
    def _open_gripper(self):
        rospy.loginfo("Opening gripper")
        self._gripper.open()
    
    def _close_gripper(self):
        rospy.loginfo("Closing gripper")
        self._gripper.close()

def main():
    rospy.init_node('simple_teleop')
    wait_for_time()
    
    teleop = GripperTeleop()
    teleop.start()
    
    rospy.loginfo("Simple teleop started")
    rospy.spin()

if __name__ == '__main__':
    main()
