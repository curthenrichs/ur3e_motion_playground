#!/usr/bin/env python

'''
Author: Curt Henrichs

Lively-IK UR driver interface node.

Interfaces with lively-ik's joint angle solutions topic to generate messages to
underlying UR driver and Robotiq gripper driver.

Adjust timing with the `joint_step_delay` param. Set to a value that should be
safe and stable. Note smaller values will make the robot move faster at the expense
of stability and safety. Larger values will increase the time it takes to reach
target which may lead to large lag in respone to goal change.
'''

import yaml
import rospy

from lively_ik.msg import JointAngles
from std_msgs.msg import Empty, Bool, Float32
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_STEP_DELAY = 0.5
DEFAULT_START_ENABLED = False


class LikUrInterfaceNode:

    def __init__(self, info_file_path, joint_step_delay, start_enabled):
        self._joint_step_delay = joint_step_delay
        self._enabled = start_enabled

        # Hardware parameters
        fin = open(info_file_path,'r')
        y = yaml.load(fin, Loader=yaml.Loader)
        self._arm_joint_names = y['joint_ordering']
        self._arm_starting_config = y['starting_config']
        fin.close()

        # ROS Publishers
        self._grip_cmd_pub = rospy.Publisher('gripper/cmd', GripperCmd, queue_size=10)
        self._traj_pub = rospy.Publisher('scaled_pos_traj_controller/command',JointTrajectory,queue_size=5)

        # ROS Subscribers
        self._grip_position_sub = rospy.Subscriber('interface/gripper_position',Float32,self._grip_position_cb)
        self._enabled_sub = rospy.Subscriber('interface/enable',Bool,self._enabled_cb)
        self._ja_sub = rospy.Subscriber('relaxed_ik/joint_angle_solutions',JointAngles, self._ja_cb)
        self._set_initial_pose_sub = rospy.Subscriber('interface/set_initial_pose',Empty, self._set_initial_pose_cb)

    def _enabled_cb(self, msg):
        print 'enabling', msg
        self._enabled = msg.data
        print self._enabled

    def _ja_cb(self, ja, enableOverride=False):
        if self._enabled or enableOverride:
            point = JointTrajectoryPoint()
            angles = list(ja.angles.data)
            point.positions = angles
            point.time_from_start = rospy.Duration.from_sec(self._joint_step_delay)
            self._arm_move(point)

    def _arm_move(self, p):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = 'world'
        names = list(self._arm_joint_names)
        traj.joint_names = names
        traj.points = [p]
        self._traj_pub.publish(traj)

    def _grip_position_sub(self, msg):
        cmd = GripperCmd()
        cmd.position = msg.data # value between 0 (closed) and 0.085 (open)
        cmd.speed = 0.1
        cmd.force = 100
        self._grip_cmd_pub.publish(cmd)

    def _set_initial_pose_cb(self, noop):
        ja = JointAngles()
        ja.angles.data = self._arm_starting_config
        self._ja_cb(ja,True)
        self._grip_position_cb(Float32(0.085))


if __name__ == "__main__":
    rospy.init_node("lik_ur_interface")

    info_file_path = rospy.get_param('~info_file_path')
    joint_step_delay = rospy.get_param('~joint_step_delay',DEFAULT_JOINT_STEP_DELAY)
    start_enabled = rospy.get_param('~start_enabled',DEFAULT_START_ENABLED)

    node = LikUrInterfaceNode(info_file_path, joint_step_delay, start_enabled)
    rospy.spin()
