#!/usr/bin/env python3

'''
'''

import rospy
import tkinter as tk
from tkinter import ttk 

from std_msgs.msg import Empty, Bool, Int8


DEFAULT_ENABLE_STATE = False
DEFAULT_POSE_MUX_INDEX = 0
DEFAULT_JOINT_MUX_INDEX = 0


class HardwareInterfaceUI:

    def __init__(self, enable_state, pose_mux_index, joint_mux_index):
        self._enable_state = enable_state

        # Create Tkinter UI
        self._gui_root = tk.Tk()
        self._gui_root.title("Hardware Interface UI")
        frame = tk.Frame(self._gui_root)
        frame.pack()

        btn_frame = tk.Frame(frame)
        btn_frame.pack()

        self._enable_button = tk.Button(btn_frame, text="Enable", command=self._enable)
        self._enable_button.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        self._initalize_button = tk.Button(btn_frame, text="Initialize", command=self._initialize)
        self._initalize_button.pack(side=tk.RIGHT, expand=True, fill=tk.X)


        pose_mux_frame = tk.Frame(frame)
        pose_mux_frame.pack()

        self._pose_mux_lbl = tk.Label(pose_mux_frame, text="Pose MUX")
        self._pose_mux_lbl.pack(side=tk.LEFT)

        self._pose_slctVar = tk.StringVar()
        self._pose_slctVar.set(pose_mux_index)
        self._pose_slctVar.trace("w", self._select_pose)

        self._pose_none_slct_btn = tk.Radiobutton(pose_mux_frame, text="No Pose", variable=self._pose_slctVar, value=0)
        self._pose_none_slct_btn.pack(side=tk.RIGHT)
        self._pose_rviz_slct_btn = tk.Radiobutton(pose_mux_frame, text="Interactive RViz", variable=self._pose_slctVar, value=1)
        self._pose_rviz_slct_btn.pack(side=tk.RIGHT)
        self._pose_path_slct_btn = tk.Radiobutton(pose_mux_frame, text="Path Reader", variable=self._pose_slctVar, value=2)
        self._pose_path_slct_btn.pack(side=tk.RIGHT)
        self._pose_static_slct_btn = tk.Radiobutton(pose_mux_frame, text="Static Pose", variable=self._pose_slctVar, value=3)
        self._pose_static_slct_btn.pack(side=tk.RIGHT)


        joint_mux_frame = tk.Frame(frame)
        joint_mux_frame.pack()

        self._joint_mux_lbl = tk.Label(joint_mux_frame, text="Joint MUX")
        self._joint_mux_lbl.pack(side=tk.LEFT)

        self._joint_slctVar = tk.StringVar()
        self._joint_slctVar.set(joint_mux_index)
        self._joint_slctVar.trace("w", self._select_js)

        self._joint_none_slct_btn = tk.Radiobutton(joint_mux_frame, text="No Joints", variable=self._joint_slctVar, value=0)
        self._joint_none_slct_btn.pack(side=tk.RIGHT)
        self._joint_ltk_slct_btn = tk.Radiobutton(joint_mux_frame, text="Lively-TK", variable=self._joint_slctVar, value=1)
        self._joint_ltk_slct_btn.pack(side=tk.RIGHT)
        self._joint_moveit_slct_btn = tk.Radiobutton(joint_mux_frame, text="MoveIT", variable=self._joint_slctVar, value=2)
        self._joint_moveit_slct_btn.pack(side=tk.RIGHT)
        self._joint_driver_slct_btn = tk.Radiobutton(joint_mux_frame, text="Driver", variable=self._joint_slctVar, value=3)
        self._joint_driver_slct_btn.pack(side=tk.RIGHT)

        # ROS interface
        self._enable_pub = rospy.Publisher('hw_interface/enable',Bool,queue_size=5)
        self._set_initial_pub = rospy.Publisher('hw_interface/set_initial_pose',Empty,queue_size=5)
        self._pose_mux_pub = rospy.Publisher('pose_mux/select',Int8, queue_size=5)
        self._joint_mux_pub = rospy.Publisher('joint_mux/select',Int8, queue_size=5)

    def _enable(self, hold=False):
        if not hold:
            self._enable_state = not self._enable_state

        self._enable_pub.publish(self._enable_state)

        if self._enable_state:
            self._enable_button['text'] = 'Disable'
        else:
            self._enable_button['text'] = 'Enable'

    def _initialize(self):
        self._set_initial_pub.publish(Empty())

    def _select_pose(self, *args):
        self._pose_mux_pub.publish(Int8(int(self._pose_slctVar.get())))

    def _select_js(self, *args):
        self._joint_mux_pub.publish(Int8(int(self._joint_slctVar.get())))

    def spin(self):

        rospy.sleep(5) # wait a bit for everything to set up

        self._enable(False)
        self._select_pose()
        self._select_js()

        rospy.on_shutdown(self._shutdown_cb)
        self._gui_root.after(250, self.check_for_shutdown)
        self._gui_root.mainloop()

    def check_for_shutdown(self):
        if rospy.is_shutdown():
            self._gui_root.destroy()
        else:
            self._gui_root.after(250, self.check_for_shutdown)

    def _shutdown_cb(self):
        try:
            self._gui_root.destroy()
        except:
            pass


if __name__ == "__main__":
    rospy.init_node("hardware_interface_ui")

    enable_state = rospy.get_param("~enable_state", DEFAULT_ENABLE_STATE)
    pose_mux_index = rospy.get_param("~pose_mux_index", DEFAULT_POSE_MUX_INDEX)
    joint_mux_index = rospy.get_param("~joint_mux_index", DEFAULT_JOINT_MUX_INDEX)

    node = HardwareInterfaceUI(enable_state, pose_mux_index, joint_mux_index)
    node.spin()
