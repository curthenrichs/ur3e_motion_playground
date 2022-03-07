#!/usr/bin/env python3

import tf
import rospy
import tkinter as tk

from geometry_msgs.msg import Pose, Vector3, Quaternion


DEFAULT_POSE_UPDATE_RATE = 1


class NumberEntryElement:

    def __init__(self, parent, labelStr, startVal):
        frame = tk.Frame(parent)
        frame.pack()

        lbl = tk.Label(frame, text=labelStr)
        lbl.pack(side=tk.LEFT)

        validation = frame.register(self.validate)

        self.entry = tk.Entry(frame, validate="key", validatecommand=(validation, '%S'))
        self.entry.insert(tk.END, str(startVal))
        self.entry.pack(side=tk.LEFT, expand=True, fill=tk.X)

    @property
    def value(self):
        return float(self.entry.get())

    def validate(self, char):
        return char.isnumeric() or char == '.' or char =='-'


class StaticPoseUI:
    
    def __init__(self, pose_update_rate):

        # Create Tkinter UI
        self._gui_root = tk.Tk()
        self._gui_root.title("Static Pose UI")

        position_frame = tk.Frame(self._gui_root)
        position_frame.pack()

        pos_lbl = tk.Label(position_frame, text="Position")
        pos_lbl.pack()

        self._pos_x = NumberEntryElement(position_frame,'x',0)
        self._pos_y = NumberEntryElement(position_frame,'y',0)
        self._pos_z = NumberEntryElement(position_frame,'z',0)

        orientation_frame = tk.Frame(self._gui_root)
        orientation_frame.pack()

        pos_lbl = tk.Label(orientation_frame, text="Orientation")
        pos_lbl.pack()

        self._ort_r = NumberEntryElement(orientation_frame,'r',0)
        self._ort_p = NumberEntryElement(orientation_frame,'p',0)
        self._ort_y = NumberEntryElement(orientation_frame,'y',0)

        # ROS interface
        self._pose_pub = rospy.Publisher("static_pose/pose", Pose, queue_size=5)

        # ROS timer for position update
        self._pose_timer = rospy.Timer(rospy.Duration(1.0/pose_update_rate), self._position_cb)
 
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

    def _position_cb(self, event):

        # pack position
        pos = Vector3(
            x=self._pos_x.value,
            y=self._pos_y.value,
            z=self._pos_z.value)

        # compute quaternion from rpy and pack
        ort = self._euler_2_quat(
            r=self._ort_r.value,
            p=self._ort_p.value,
            y=self._ort_y.value)

        # send
        msg = Pose(position=pos, orientation=ort)
        self._pose_pub.publish(msg)

    def _euler_2_quat(self, r, p , y):
        (qx,qy,qz,qw) = tf.transformations.quaternion_from_euler(r,p,y)
        return Quaternion(x=qx,y=qy,z=qz,w=qw)


if __name__ == "__main__":
    rospy.init_node("static_pose_ui")

    pose_update_rate = rospy.get_param('~pose_update_rate',DEFAULT_POSE_UPDATE_RATE)

    node = StaticPoseUI(pose_update_rate)
    node.spin()
