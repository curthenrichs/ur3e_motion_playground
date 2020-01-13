#!/usr/bin/env python

'''
Author: Curt Henrichs

Used to verify correct operation of UR position controller and Gripper driver
before integration into Lively-IK.
'''

import yaml
import rospy

from robotiq_85_msgs.msg import GripperCmd, GripperStat
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DirectJointNode:

    def __init__(self, info_file_path):

        # Hardware parameters
        fin = open(info_file_path,'r')
        y = yaml.load(fin, Loader=yaml.Loader)
        self._arm_joint_names = y['joint_ordering']
        self._arm_starting_config = y['starting_config']
        self._fixed_frame = y['fixed_frame']
        self._gripper_starting_config = 0.085
        fin.close()

        # ROS Publishers
        self._traj_pub = rospy.Publisher('scaled_pos_traj_controller/command',JointTrajectory,queue_size=5)
        self._grip_cmd_pub = rospy.Publisher('gripper/cmd', GripperCmd, queue_size=10)

    def spin(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print "Spin!"

            point = JointTrajectoryPoint()
            point.positions = self._arm_starting_config
            point.time_from_start = rospy.Duration.from_sec(1)
            self._arm_move(point)

            self._grip_cmd(self._gripper_starting_config)

            print "\n"

            rate.sleep()

    def _arm_move(self, p):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = self._fixed_frame
        traj.joint_names = self._arm_joint_names
        traj.points = [p]
        self._traj_pub.publish(traj)

        print "Arm Joints:", p.positions

    def _grip_cmd(self, ja):
        cmd = GripperCmd()
        cmd.position = ja
        cmd.speed = 0.02
        cmd.force = 100.0
        self._grip_cmd_pub.publish(cmd)

        print "Gripper Pos:", ja


if __name__ == "__main__":
    rospy.init_node("direct_joint_node")

    info_file_path = rospy.get_param('~info_file_path')

    node = DirectJointNode(info_file_path)
    node.spin()
