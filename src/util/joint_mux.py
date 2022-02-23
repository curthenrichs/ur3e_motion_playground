#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int8
from sensor_msgs.msg import JointState


class JointMux:

    def __init__(self, arm_topics=[], grip_topics=[]):

        if len(arm_topics) != len(grip_topics):
            raise ValueError('Arrays must match in length')

        self._slct_option = 0

        self._out_arm_pub = rospy.Publisher('joint_mux/output_arm', JointState, queue_size=5)
        self._out_grip_pub = rospy.Publisher('joint_mux/output_gripper', JointState, queue_size=5)
        self._slct_sub = rospy.Subscriber('joint_mux/select', Int8, self._slct_cb)

        self._in_arm_subscriber_topics = arm_topics
        self._in_arm_subscriber_ros = []
        self._in_arm_subscriber_joints = []

        for i in range(0,len(self._in_arm_subscriber_topics)):
            topic = self._in_arm_subscriber_topics[i]
            self._in_arm_subscriber_ros.append(rospy.Subscriber(topic, JointState, lambda msg: self._joint_arm_cb(msg, i)))
           
            joints = JointState()
            self._in_arm_subscriber_joints.append(joints)

        self._in_grip_subscriber_topics = grip_topics
        self._in_grip_subscriber_ros = []
        self._in_grip_subscriber_joints = []

        for i in range(0,len(self._in_grip_subscriber_topics)):
            topic = self._in_grip_subscriber_topics[i]
            self._in_grip_subscriber_ros.append(rospy.Subscriber(topic, JointState, lambda msg: self._joint_grip_cb(msg, i)))
           
            joints = JointState()
            self._in_grip_subscriber_joints.append(joints)

    def _joint_arm_cb(self, msg, idx):
        self._in_arm_subscriber_joints[idx] = msg

        if idx +1 == self._slct_option:
            self._out_arm_pub.publish(msg)

    def _joint_grip_cb(self, msg, idx):
        self._in_grip_subscriber_joints[idx] = msg

        if idx +1 == self._slct_option:
            self._out_grip_pub.publish(msg)

    def _slct_cb(self, msg):
        if msg.data < 0 or msg.data > len(self._in_arm_subscriber_topics):
            return # Ignore since its out of bounds

        self._slct_option = msg.data

        if self._slct_option > 0:
            arm_msg = self._in_arm_subscriber_joints[self._slct_option - 1]
            self._out_arm_pub.publish(arm_msg)

            grip_msg = self._in_grip_subscriber_joints[self._slct_option - 1]
            self._out_grip_pub.publish(grip_msg)
 

if __name__ == "__main__":
    rospy.init_node("joint_mux")

    arm_topics = [
        'hw_interface/fake_joint_state',
        'move_group/fake_controller_joint_states',
        'driver/joint_state'
    ]

    grip_topics = [
        'hw_interface/fake_joint_state',
        'hw_interface/fake_joint_state',
        'gripper/joint_state'
    ]

    node = JointMux(arm_topics, grip_topics)
    rospy.spin()