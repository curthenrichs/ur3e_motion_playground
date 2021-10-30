#!/usr/bin/env python


import tf
import rospy

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Vector3, Quaternion, Pose

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


class InteractiveMarkerController:

    def __init__(self):
        self._ee_link = 'ee_link'
        self._grip_pose = Pose(position=Vector3(),orientation=Quaternion(0,0,0,1))

        # Marker
        self._marker_server = InteractiveMarkerServer("robot_controls")
        self._target_marker = self._make_target_marker()
        self._marker_server.insert(self._target_marker, self._marker_feedback)
        self._marker_server.applyChanges()

        self._ee_goal_pub = rospy.Publisher('lively_tk/ee_pose_goal',Pose,queue_size=10)
        self._enable_pub = rospy.Publisher('hw_interface/enable',Bool,queue_size=10)
        self._set_initial_pub = rospy.Publisher('hw_interface/set_initial_pose',Empty,queue_size=10)

        self._tf_sub = tf.TransformListener()

    def _marker_feedback(self, feedback):
        print "Marker Feedback:", feedback, "\n\n"
        self._target_marker.pose = feedback.pose

    def spin(self):

        # Initial startup time before configuring robot
        print 'waiting for system ready'
        rospy.sleep(5)

        # Setting robot to home state
        self._set_initial_pub.publish(Empty())

        # wait for robot to read start state
        print 'letting robot settle at start state'
        rospy.sleep(15)

        # set marker and relaxed ik to default
        print 'setting marker and letting relaxed ik settle'

        (pos, rot) = self._tf_sub.lookupTransform('world','ee_link',rospy.Time(0))
        eePose = Pose(
            position=Vector3(
                x=pos[0],
                y=pos[1],
                z=pos[2]),
            orientation=Quaternion(
                x=rot[0],
                y=rot[1],
                z=rot[2],
                w=rot[3]))
        print 'EE Pose', eePose

        self._target_marker.pose = eePose
        self._marker_server.setPose(self._target_marker.name, self._target_marker.pose)
        self._marker_server.applyChanges()

        # make sure queue is filled with good pose?
        for i in range(0,20):
            msg = EEPoseGoals()
            if self._separate_grip_chain:
                msg.ee_poses = [eePose, self._grip_pose]
            else:
                msg.ee_poses = [eePose]
            self._ee_goal_pub.publish(msg)

        rospy.sleep(5)
        self._enable_pub.publish(Bool(True))

        # relaxed ik control loop
        print 'Starting marker publish loop'
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            msg = EEPoseGoals()
            if self._separate_grip_chain:
                msg.ee_poses = [self._target_marker.pose, self._grip_pose]
            else:
                msg.ee_poses = [self._target_marker.pose]
            self._ee_goal_pub.publish(msg)

            rate.sleep()

    def _make_marker(self):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.scale.x = 0.125
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

    def _make_target_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = "world"
        marker.pose.position = Vector3(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.scale = 0.25
        marker.name = "pose target"
        marker.description = "Target EE pose for Arm"

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self._make_marker())
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        return marker


if __name__ == "__main__":
    rospy.init_node("interactive_marker_controller")
    node = InteractiveMarkerController()
    node.spin()
