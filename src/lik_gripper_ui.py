#!/usr/bin/env python

'''
Author: Curt Henrichs

Simple GUI to provide gripper commands to the lively-ik direct control for testing
purposes.

std_msgs/Header header
float32[] dc_values
'''

import yaml
import rospy
import Tkinter as tk

from lively_ik.msg import DCPoseGoals


DEFAULT_POSITION_UPDATE_RATE = 30.0


class LikGripperUI:

    def __init__(self, info_file_path, grip_joint, update_rate):

        # Hardware parameters
        fin = open(info_file_path,'r')
        y = yaml.load(fin, Loader=yaml.Loader)
        self._joint_names = y['joint_ordering']
        self._starting_config = y['starting_config']
        fin.close()

        self._grip_idx = None
        for i in range(0,len(self._joint_names)):
            if self._joint_names[i] == grip_joint:
                self._grip_idx = i

        # Create Tkinter UI
        self._gui_root = tk.Tk()
        frame = tk.Frame(self._gui_root)
        frame.pack()

        self._close_button = tk.Button(frame, text="Close",command=self._close_gripper)
        self._close_button.pack(side=tk.LEFT)
        self._open_button = tk.Button(frame, text="Open",command=self._open_gripper)
        self._open_button.pack(side=tk.RIGHT)

        self._position_slider = tk.Scale(frame, from_=0, to=100, orient=tk.HORIZONTAL)
        self._position_slider.set(self._starting_config[self._grip_idx] / 0.085 * 100)
        self._position_slider.pack()

        # ROS Interface
        self._dc_pose_goal_pub = rospy.Publisher('relaxed_ik/dc_pose_goals',DCPoseGoals,queue_size=10)

        # ROS timer for position update
        self._position_timer = rospy.Timer(rospy.Duration(1.0/position_update_rate), self._position_cb)

    def _open_gripper(self):
        self._position_slider.set(100)

    def _close_gripper(self):
        self._position_slider.set(0)

    def _position_cb(self, event):
        posRaw = self._position_slider.get()
        posJoint = posRaw * 0.085 / 100.0

        msg = DCPoseGoals()
        msg.dc_values = [0]*len(self._joint_names)
        msg.dc_values[self._grip_idx] = posJoint

        self._dc_pose_goal_pub.publish(msg)

    def spin(self):
        rospy.on_shutdown(self._shutdown_cb)
        self._gui_root.mainloop()

    def _shutdown_cb(self):
        self._gui_root.destroy()


if __name__ == "__main__":
    rospy.init_node("lik_gripper_ui")

    info_file_path = rospy.get_param('~info_file_path')
    gripper_joint = rospy.get_param('~gripper_joint')
    position_update_rate = rospy.get_param('~position_update_rate',DEFAULT_POSITION_UPDATE_RATE)

    node = LikGripperUI(info_file_path, gripper_joint, position_update_rate)
    node.spin()
