#!/usr/bin/env python3

'''
https://pypi.org/project/lively-tk/
'''

import time
import json
import rospy

from lively_tk import Solver, PositionMatchObjective, OrientationMatchObjective, \
                      SmoothnessMacroObjective, CollisionAvoidanceObjective,     \
                      State, Transform, ScalarRange, BoxShape, Translation,      \
                      Rotation

from std_msgs.msg import Empty
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
        self._initial_weights = {"EE Position": 50, "EE Rotation": 25, "General Smoothness": 10, "Collision Avoidance": 10}
        self._js_config = {jn: 0 for jn in self._joint_names}

        # Lively Solver
        self._solver = Solver(
            urdf=urdf,
            objectives=[
                PositionMatchObjective(name="EE Position",link=ee_frame,weight=self._initial_weights["EE Position"]),
                OrientationMatchObjective(name="EE Rotation",link=ee_frame,weight=self._initial_weights["EE Rotation"]),
                SmoothnessMacroObjective(name="General Smoothness",weight=self._initial_weights["General Smoothness"]),
                CollisionAvoidanceObjective(name="Collision Avoidance",weight=self._initial_weights["Collision Avoidance"])
            ], 
            root_bounds=[
                ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0), # Translational
                ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0),ScalarRange(value=0.0,delta=0.0)  # Rotational
            ],
            shapes=[], 
            initial_state=State(origin=Transform.identity(),joints=self._initial_config), # Optional
            only_core=False, # Only use this flag if you are not using liveliness objectives and want a slight speed-up.
            max_retries=1, # Number of times the solution is attempted (default 1)
            max_iterations=150 # Number of iterations per try (default 150)
        )

        print('\n\n\n')
        print(urdf)
        print('\n\n\n')
        print(dir(self._solver),'\n')
        print('---')
        print(self._solver.current_goals)
        print('---')
        print(dir(self._solver.current_state))
        print(self._solver.current_state.center_of_mass)
        print(self._solver.current_state.frames)
        print(self._solver.current_state.joints)
        print(self._solver.current_state.origin)
        print(self._solver.current_state.proximity)
        print('---')
        print(self._solver.joints)
        print(self._solver.links)
        print(self._solver.objectives)
        print('\n\n\n')

        # ROS Interface
        self._ja_pub = rospy.Publisher('hw_interface/target_joint_state', JointState, queue_size=10)
        self._pose_sub = rospy.Subscriber('pose_mux/output', Pose, self._pose_cb)
        self._reset_init_sub = rospy.Subscriber('lively/reset_init', Empty, self._reset_init_cb)
        self._reset_js_sub = rospy.Subscriber('lively/reset_joint_state', Empty, self._reset_js_cb)
        self._joint_sub = rospy.Subscriber('joint_states', JointState, self._joint_cb)

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._js_config.keys():
                self._js_config[name] = pos

    def _pose_cb(self, msg):
        #print("New Target", msg)
        self._target = msg

    def _reset_init_cb(self, _):
        self._solver.reset(state=State(origin=Transform.identity(),joints=self._initial_config))

    def _reset_js_cb(self, _):
        self._solver.reset(state=State(origin=Transform.identity(),joints=self._js_config))

    def solver_step(self, pose, current_time=0):
        jNames = list(self._joint_names)

        #print(pose)
        #print(self._initial_weights.values())
        #print(current_time)

        state = self._solver.solve(
            goals=[
                Translation(x=pose.position.x, y=pose.position.y, z=pose.position.z),
                Rotation(w=pose.orientation.w, x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z),
                None,
                None
            ],
            weights=list(self._initial_weights.values()),
            time=0,
            shapes=[]
        )

        #print(state.joints)

        jPos = [state.joints[n] for n in jNames]
        
        return jPos, jNames

    def spin(self):
        rate = rospy.Rate(self._spin_rate)

        start_time = time.time() 

        self._reset_init_cb(None)

        while not rospy.is_shutdown():
            #print('Solver loop')

            current_time = time.time() - start_time

            positions, names = self.solver_step(self._target, current_time)

            msg = JointState()
            msg.position = positions
            msg.name = names
            self._ja_pub.publish(msg)
            #print(msg)

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