#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose


class PoseMux:

    def __init__(self, topics=[]):
        self._slct_option = 0

        self._out_pub = rospy.Publisher('pose_mux/output', Pose, queue_size=5)
        self._slct_sub = rospy.Subscriber('pose_mux/select', Int8, self._slct_cb)

        self._in_subscriber_topics = topics
        self._in_subscriber_ros = []
        self._in_subscriber_poses = []

        for i in range(0,len(self._in_subscriber_topics)):
            topic = self._in_subscriber_topics[i]
            self._in_subscriber_ros.append(rospy.Subscriber(topic, Pose, lambda msg: self._pose_cb(msg, i)))
           
            pose = Pose()
            pose.orientation.w = 1
            self._in_subscriber_poses.append(pose)

    def _pose_cb(self, msg, idx):
        self._in_subscriber_poses[idx] = msg

        if idx + 1 == self._slct_option:
            self._out_pub.publish(msg)

    def _slct_cb(self, msg):
        if msg.data < 0 or msg.data > len(self._in_subscriber_topics):
            return # Ignore since its out of bounds

        self._slct_option = msg.data

        if self._slct_option > 0:
            pose = self._in_subscriber_poses[self._slct_option - 1]
            self._out_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("pose_mux")

    topics = [
        'interactive_marker/pose',
        'path_reader/pose',
        'static_pose/pose'
    ]

    node = PoseMux(topics)
    rospy.spin()