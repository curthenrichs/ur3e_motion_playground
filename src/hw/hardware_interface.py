#!/usr/bin/env python3

'''
Author: Curt Henrichs

Lively-IK UR driver interface node.

Interfaces with lively-ik's joint angle solutions topic to generate messages to
underlying UR driver and Robotiq gripper driver.

Adjust timing with the `arm_joint_step_delay` param. Set to a value that should be
safe and stable. Note smaller values will make the robot move faster at the expense
of stability and safety. Larger values will increase the time it takes to reach
target which may lead to large lag in respone to goal change.
'''

import yaml
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Bool, Float32
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_ARM_JOINT_STEP_DELAY = 0.5
DEFAULT_START_ENABLED = False
DEFAULT_GRIPPER_JOINT_UPDATE_RATE = 1.0
DEFAULT_GRIPPER_SPEED = 0.1
DEFAULT_GRIPPER_EFFORT = 100


class HardwareInterfaceNode:

    def __init__(self, arm_joints, grip_joint, joint_step_delay,
                 gripper_speed, gripper_effort, start_enabled):
        self._joint_step_delay = joint_step_delay
        self._gripper_speed = gripper_speed
        self._gripper_effort = gripper_effort
        self._enabled = start_enabled

        self._arm_joint_names = arm_joints
        self._starting_config = [0 for i in range(0,len(arm_joints))]
        self._grip_joint_name = grip_joint
        self._grip_joint_value = 0
        self._update_gripper = False

        # ROS Publishers
        self._grip_cmd_pub = rospy.Publisher('gripper/cmd', GripperCmd, queue_size=10)
        self._traj_pub = rospy.Publisher('scaled_pos_traj_controller/command', JointTrajectory, queue_size=5)
        self._ja_pub = rospy.Publisher('hw_interface/fake_joint_state', JointState, queue_size=5)
        self._grip_driver_ja_pub = rospy.Publisher('hw_interface/gripper_joint', JointState, queue_size=5)

        # ROS Subscribers
        self._enabled_sub = rospy.Subscriber('hw_interface/enable', Bool,self._enabled_cb)
        self._ja_sub = rospy.Subscriber('hw_interface/target_joint_state', JointState, self._ja_cb)
        self._set_initial_pose_sub = rospy.Subscriber('hw_interface/set_initial_pose', Empty, self._set_initial_pose_cb)
        self._grip_stat_sub = rospy.Subscriber('gripper/stat', GripperStat, self._grip_stat_cb)

        self._open_pub = rospy.Subscriber('hw_interface/gripper_open',Empty,self._grip_open_cb)
        self._close_pub = rospy.Subscriber('hw_interface/gripper_close',Empty,self._grip_close_cb)
        self._position_pub = rospy.Subscriber('hw_interface/gripper_position',Float32,self._grip_pos_cb)

    def _enabled_cb(self, msg):
        self._enabled = msg.data

    def _ja_cb(self, msg, enableOverride=False):
        if self._enabled or enableOverride:
            self._ja_pub.publish(msg) # repost to fake robot data

            print(msg)

            point = JointTrajectoryPoint()
            angles = list(msg.position)

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

    def _grip_position(self, pos, enableOverride=False):
        if self._enabled or enableOverride:
            cmd = GripperCmd()
            cmd.position = pos # value between 0 (closed) and 0.085 (open)
            cmd.speed = self._gripper_speed
            cmd.force = self._gripper_effort
            self._grip_cmd_pub.publish(cmd)

    def _set_initial_pose_cb(self, noop):
        ja = JointState()
        ja.name = list(self._arm_joint_names)
        ja.position = list(self._starting_config)
        self._ja_cb(ja,True)

    def _grip_stat_cb(self, msg):
        ja = JointState()
        ja.name = [self._grip_joint_name]
        ja.position = [msg.position * 10]

        self._grip_driver_ja_pub.publish(ja)

    def _grip_open_cb(self, _):
        self._grip_position(_grip_clamp(100))

    def _grip_close_cb(self, _):
        self._grip_position(self._grip_clamp(0))

    def _grip_pos_cb(self, msg):
        self._grip_position(self._grip_clamp(msg.data))
        
    def _grip_clamp(self, x):
        y = (0.085 / 100.0) * x
        
        if y > 0.085:
            y = 0.085
        elif y < 0:
            y = 0

        return y


if __name__ == "__main__":
    rospy.init_node("hardware_interface")

    arm_joints = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    ]

    gripper_joint = 'robotiq_85_left_knuckle_joint'
    
    arm_joint_step_delay = rospy.get_param('~arm_joint_step_delay',DEFAULT_ARM_JOINT_STEP_DELAY)
    gripper_speed = rospy.get_param('~gripper_speed',DEFAULT_GRIPPER_SPEED)
    gripper_effort = rospy.get_param('~gripper_effort',DEFAULT_GRIPPER_EFFORT)
    start_enabled = rospy.get_param('~start_enabled',DEFAULT_START_ENABLED)

    node = HardwareInterfaceNode(arm_joints, gripper_joint, arm_joint_step_delay,
                                 gripper_speed, gripper_effort, start_enabled)
    rospy.spin()
