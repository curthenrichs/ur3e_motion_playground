#!/usr/bin/env python3

import rospy
import tkinter as tk

from std_msgs.msg import Empty


DEFAULT_WEIGHT_UPDATE_RATE = 1

### self._reset_sub = rospy.Subscriber('lively/reset', Empty, self.reset_solver)

class LivelyUI:
    
    def __init__(self, weight_update_rate):
        
        # Create Tkinter UI
        self._gui_root = tk.Tk()
        self._gui_root.title("Lively UI")
        frame = tk.Frame(self._gui_root)
        frame.pack()

        btn_frame = tk.Frame(frame)
        btn_frame.pack()

        self._reset_init_button = tk.Button(btn_frame, text="Reset to Initial", command=self._reset_init)
        self._reset_init_button.pack(side=tk.LEFT, expand=True, fill=tk.X)

        self._reset_js_button = tk.Button(btn_frame, text="Reset to Joint State", command=self._reset_js)
        self._reset_js_button.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # ROS interface 
        self._reset_init_pub = rospy.Publisher('lively/reset_init', Empty, queue_size=10)
        self._reset_js_pub = rospy.Publisher('lively/reset_joint_state', Empty, queue_size=10)

        # ROS timer for position update
        self._weight_timer = rospy.Timer(rospy.Duration(1.0/weight_update_rate), self._weights_cb)

    def spin(self):
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

    def _weights_cb(self, event):
        pass #TODO in the future we may want to adjust the weights

    def _reset_init(self):
        self._reset_init_pub.publish(Empty())

    def _reset_js(self):
        self._reset_js_pub.publish(Empty())


if __name__ == "__main__":
    rospy.init_node("lively_ui")

    weight_update_rate = rospy.get_param('~weight_update_rate',DEFAULT_WEIGHT_UPDATE_RATE)

    node = LivelyUI(weight_update_rate)
    node.spin()
