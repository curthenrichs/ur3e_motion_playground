#!/usr/bin/env python3

'''
Author: Curt Henrichs

Simple GUI to provide gripper commands to the driver interface for testing
purposes.
'''

import rospy
import tkinter as tk

from std_msgs.msg import Empty, Float32


DEFAULT_POSITION_UPDATE_RATE = 30.0


class GripperUI:

    def __init__(self, position_update_rate):

        # Create Tkinter UI
        self._gui_root = tk.Tk()
        self._gui_root.title("Gripper UI")
        frame = tk.Frame(self._gui_root)
        frame.pack()

        self._close_button = tk.Button(frame, text="Close",command=self._close_gripper)
        self._close_button.pack(side=tk.LEFT)
        self._open_button = tk.Button(frame, text="Open",command=self._open_gripper)
        self._open_button.pack(side=tk.RIGHT)

        self._position_slider = tk.Scale(frame, from_=0, to=100, orient=tk.HORIZONTAL)
        self._position_slider.set(100)
        self._position_slider.pack()

        # ROS interface
        self._open_pub = rospy.Publisher('hw_interface/gripper_open',Empty,queue_size=5)
        self._close_pub = rospy.Publisher('hw_interface/gripper_close',Empty,queue_size=5)
        self._position_pub = rospy.Publisher('hw_interface/gripper_position',Float32,queue_size=5)

        # ROS timer for position update
        self._position_timer = rospy.Timer(rospy.Duration(1.0/position_update_rate), self._position_cb)

    def _open_gripper(self):
        self._position_slider.set(100)

    def _close_gripper(self):
        self._position_slider.set(0)

    def _position_cb(self, event):
        posRaw = self._position_slider.get()
        posJoint = posRaw * 0.085 / 100.0

        self._position_pub.publish(Float32(posJoint))

    def spin(self):
        rospy.on_shutdown(self._shutdown_cb)
        self._gui_root.mainloop()

    def _shutdown_cb(self):
        try:
            self._gui_root.destroy()
        except:
            pass


if __name__ == "__main__":
    rospy.init_node("gripper_ui")

    position_update_rate = rospy.get_param('~position_update_rate',DEFAULT_POSITION_UPDATE_RATE)

    node = GripperUI(position_update_rate)
    node.spin()
