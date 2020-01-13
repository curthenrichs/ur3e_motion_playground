#!/usr/bin/env python

'''
Author: Curt Henrichs

Lively-IK based driver node.

Currently a near duplicate of the Relaxed-IK based driver node with only
difference being the message source from Relaxed-Ik to Lively-IK.

This node provides an interactive marker server to control the end-effector pose
of the robot.

This node has an open-loop setup process where it moves the robot into starting
configuration then sets pose of the interactive marker server.

Adjust timing with the JOINT_STEP_DELAY constant. Set to a value that should be
safe and stable. Note smaller values will make the robot move faster at the expense
of stability and safety. Larger values will increase the time it takes to reach
target which may lead to large lag in respone to goal change.
'''

import tf
import yaml
import rospy

from std_msgs.msg import Empty
from lively_ik.msg import JointAngles, EEPoseGoals
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from geometry_msgs.msg import Vector3, Quaternion, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


JOINT_STEP_DELAY = 0.5


class DriverTestNode:

    def __init__(self, info_file_path):
        self._enabled = False

        # Hardware parameters
        fin = open(info_file_path,'r')
        y = yaml.load(fin, Loader=yaml.Loader)
        self._arm_joint_names = y['joint_ordering']
        self._arm_starting_config = y['starting_config']
        self._ee_link = 'ee_link'
        fin.close()

        # Marker
        self._marker_server = InteractiveMarkerServer("robot_controls")
        self._target_marker = self._make_target_marker()
        self._marker_server.insert(self._target_marker, self._marker_feedback)
        self._marker_server.applyChanges()

        # ROS Publishers
        self._grip_cmd_pub = rospy.Publisher('gripper/cmd', GripperCmd, queue_size=10)
        self._ee_goal_pub = rospy.Publisher('relaxed_ik/ee_pose_goals',EEPoseGoals,queue_size=10)
        self._traj_pub = rospy.Publisher('scaled_pos_traj_controller/command',JointTrajectory,queue_size=5)

        # ROS Subscribers
        self._tf_sub = tf.TransformListener()
        self._ja_sub = rospy.Subscriber('relaxed_ik/joint_angle_solutions',JointAngles, self._ja_cb)
        self._gripper_open_sub = rospy.Subscriber('driver_test/gripper_open',Empty,self._gripper_open_cb)
        self._gripper_close_sub = rospy.Subscriber('driver_test/gripper_close',Empty,self._gripper_close_cb)

    def _marker_feedback(self, feedback):
        print "Marker Feedback:", feedback, "\n\n"
        self._target_marker.pose = feedback.pose

    def _gripper_open_cb(self, msg):
        self._grip_cmd(0.085)

    def _gripper_close_cb(self, msg):
        self._grip_cmd(0.0)

    def _ja_cb(self, ja):
        if self._enabled:
            point = JointTrajectoryPoint()
            point.positions = ja.angles.data
            point.time_from_start = rospy.Duration.from_sec(JOINT_STEP_DELAY)
            self._arm_move(point)

    def _arm_move(self, p):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = 'world'
        traj.joint_names = self._arm_joint_names
        traj.points = [p]
        self._traj_pub.publish(traj)

    def _grip_cmd(self, ja):
        cmd = GripperCmd()
        cmd.position = ja
        cmd.speed = 0.1
        cmd.force = 100
        self._grip_cmd_pub.publish(cmd)

    def spin(self):

        # Initial startup time before configuring robot
        print 'waiting for system ready'
        rospy.sleep(5)

        # setup pose loop
        print 'commanding robot'
        point = JointTrajectoryPoint()
        point.positions = self._arm_starting_config
        point.time_from_start = rospy.Duration.from_sec(10)
        self._arm_move(point)

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
            msg.ee_poses = [eePose]
            self._ee_goal_pub.publish(msg)

        rospy.sleep(5)
        self._enabled = True

        # relaxed ik control loop
        print 'Starting marker publish loop'
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            msg = EEPoseGoals()
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
        marker.description = "Targe EE pose for Arm"

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

        print marker
        return marker


if __name__ == "__main__":
    rospy.init_node("driver_test_node")

    info_file_path = rospy.get_param('~info_file_path')

    node = DriverTestNode(info_file_path)
    node.spin()
