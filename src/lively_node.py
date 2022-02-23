#!/usr/bin/env python3

'''
https://pypi.org/project/lively-tk/
'''

import json
import rospy

from lively_tk import Solver, PositionMatchObjective, OrientationMatchObjective, SmoothnessMacroObjective, CollisionAvoidanceObjective, State, Transform, ScalarRange, BoxShape

from geometry_msgs.msg import Vector3, Quaternion, Pose
from sensor_msgs.msg import JointState


DEFAULT_SPIN_RATE = 10


class LivelyNode:
    
    def __init__(self, urdf, joint_names, ee_frame, spin_rate):
        # State
        self._spin_rate = spin_rate
        self._target = Pose()
        self._target.orientation.w = 1
        self._joint_names = joint_names

        self._initial_config = {jn: 0 for jn in self._joint_names}

        # Lively Solver
        self._solver = Solver(
            urdf=urdf,
            objectives=[
                PositionMatchObjective(name="EE Position",link=ee_frame,weight=50),
                OrientationMatchObjective(name="EE Rotation",link=ee_frame,weight=25),
                SmoothnessMacroObjective(name="General Smoothness",weight=10),
                CollisionAvoidanceObjective(name="Collision Avoidance",weight=10)
            ], 
            root_bounds=[
                ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0), # Translational
                ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0)  # Rotational
            ],
            shapes=[], 
            initial_state=State(origin=Transform.identity(),joints=initial_config), # Optional
            only_core=False, # Only use this flag if you are not using liveliness objectives and want a slight speed-up.
            max_retries=1, # Number of times the solution is attempted (default 1)
            max_iterations=150 # Number of iterations per try (default 150)
        )

        # ROS Interface
        self._ja_pub = rospy.Publisher('hw_interface/joint_state', JointState, queue_size=10)
        self._pose_sub = rospy.Subscriber('pose_mux/output', Pose, self._pose_cb)

    def _pose_cb(self, msg):
        self._target = msg

    def reset_solver(self):
        self._solver.reset(state=State(origin=Transform.identity(),joints=self._initial_config))

    def solver_step(self, pose):
        jNames = list(self._joint_names)

        state = self._solver.solve(goals=[
            Translation(x=pose.position.x, y=pose.position.y, z=pose.position.z),
            Rotation(w=pose.orientation.w, x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z)
        ])

        jPos = [state.joints[n] for n in jNames]
        
        return jPos, jNames

    def spin(self):
        rate = rospy.Rate(self._spin_rate)

        while not rospy.is_shutdown():

            msg = JointState()
            positions, names = self.solver_step(self._target)
            msg.position = positions
            msg.name = names

            self._ja_pub.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lively_node")

    joint_names = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    ]

    ee_frame = 'ee_link'

    urdf = rospy.get_param('robot_description')
    spin_rate = rospy.get_param('~spin_rate',DEFAULT_SPIN_RATE)

    node = LivelyNode(urdf,joint_names,ee_frame,spin_rate)
    node.spin()