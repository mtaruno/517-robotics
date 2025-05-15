#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker, 
    InteractiveMarkerControl, 
    InteractiveMarkerFeedback,
    Marker
)
from geometry_msgs.msg import Point
import math
from robot_api.base import Base

class BaseMarkers(object):
    """Controls the mobile base using interactive markers"""
    
    def __init__(self):
        self._server = InteractiveMarkerServer('base_markers')
        self._base = Base()
        
        # Create markers for forward/backward and rotation
        self._create_forward_marker()
        self._create_turn_markers()
        
        # 'Publish' the markers
        self._server.applyChanges()

    def _create_forward_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.name = 'forward_marker'
        int_marker.description = 'Move 0.5m forward'
        int_marker.scale = 0.3

        # Create a sphere marker
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        # Create control
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.markers.append(marker)
        
        # Position the marker 0.5m in front of the robot
        int_marker.pose.position.x = 0.5
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.3
        
        int_marker.controls.append(control)
        
        self._server.insert(int_marker, self._forward_callback)

    def _create_turn_markers(self):
        # Create markers for left and right rotation
        for direction in ['left', 'right']:
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = 'base_link'
            int_marker.name = f'turn_{direction}_marker'
            int_marker.description = f'Turn {direction}'
            int_marker.scale = 0.3

            # Create an arrow marker
            marker = Marker()
            marker.type = Marker.ARROW
            marker.scale.x = 0.3
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.5
            marker.color.g = 0.0
            marker.color.b = 0.5
            marker.color.a = 1.0

            # Create control
            control = InteractiveMarkerControl()
            control.interaction_mode = InteractiveMarkerControl.BUTTON
            control.markers.append(marker)
            
            # Position the markers on either side of the robot
            int_marker.pose.position.x = 0.0
            int_marker.pose.position.y = 0.3 if direction == 'left' else -0.3
            int_marker.pose.position.z = 0.3
            
            # Orient the arrows to show rotation direction
            if direction == 'left':
                int_marker.pose.orientation.w = math.cos(math.pi/4)
                int_marker.pose.orientation.z = math.sin(math.pi/4)
            else:
                int_marker.pose.orientation.w = math.cos(-math.pi/4)
                int_marker.pose.orientation.z = math.sin(-math.pi/4)
            
            int_marker.controls.append(control)
            
            self._server.insert(int_marker, self._turn_callback)

    def _forward_callback(self, feedback):
        """Called when the forward marker is clicked."""
        rospy.loginfo(f'Forward callback received event type: {feedback.event_type}')
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo('Moving forward 0.5m')
            self._base.go_forward(0.5)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo('Mouse down on forward marker')
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo('Mouse up on forward marker')

    def _turn_callback(self, feedback):
        """Called when a turn marker is clicked."""
        rospy.loginfo(f'Turn callback received event type: {feedback.event_type}')
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            direction = 1 if 'left' in feedback.marker_name else -1
            angle = direction * math.pi/6  # Turn 30 degrees
            rospy.loginfo(f'Turning {angle} radians')
            self._base.turn(angle)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo('Mouse down on turn marker')
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo('Mouse up on turn marker')

def main():
    rospy.init_node('base_markers')
    base_markers = BaseMarkers()
    rospy.spin()

if __name__ == '__main__':
    main()
