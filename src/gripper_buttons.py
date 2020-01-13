#!/usr/bin/env python

'''
Author: Curt Henrichs

Simple GUI to provide open/close commands to the driver interface for testing
purposes.
'''

import rospy
import Tkinter as tk

from std_msgs.msg import Empty


class GripperButtonsGUI:

    def __init__(self):

        # ROS interface
        self._open_pub = rospy.Publisher('driver_test/gripper_open',Empty,queue_size=5)
        self._close_pub = rospy.Publisher('driver_test/gripper_close',Empty,queue_size=5)

        # Create UI
        self._gui_root = tk.Tk()
        frame = tk.Frame(self._gui_root)
        frame.pack()

        open_button = tk.Button(frame, text="Open",command=self._open_gripper)
        open_button.pack(side=tk.LEFT)
        close_button = tk.Button(frame, text="Close",command=self._close_gripper)
        close_button.pack(side=tk.LEFT)

    def _open_gripper(self):
        self._open_pub.publish(Empty())

    def _close_gripper(self):
        self._close_pub.publish(Empty())

    def spin(self):
        self._gui_root.mainloop()


if __name__ == "__main__":
    rospy.init_node("gripper_buttons")

    node = GripperButtonsGUI()
    node.spin()
