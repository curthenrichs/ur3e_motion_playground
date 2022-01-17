#!/usr/bin/env python

'''
MIT License

Copyright (c) 2019 Curt Henrichs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

'''
Path Reader MoveIt Node

Reads and operates on a path saved with the Path Saver Node using Moveit. All
control is handled by MoveIt Commander and the MoveIt ROS subsystem. Robot will
move to each pose recorded in the path then will move to the next looping back
to first point if repeat is set to true.

Parameters:
    - ~filepath
        "Path Pose JSON file to execute"
    - ~repeat (false)
        "Loop over path, if not provided then (default)"
    - ~move_group (manipulator)
        "Robot arm's move_group configuration name, if not provided then (default)"
'''

import sys
import json
import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Vector3, Quaternion


DEFAULT_MOVE_GROUP = 'manipulator'


class PathReaderMoveit:

    def __init__(self, filepath, repeat, move_group):
        self._repeat = repeat

        file = open(filepath,'r')
        self._data = json.load(file)
        file.close()

        self._move_group = moveit_commander.MoveGroupCommander(move_group)

    def spin(self):

        rospy.sleep(10)
        raw_input("Ready to start movement? (press enter)")

        while not rospy.is_shutdown():

            # run path
            count = 1
            for p in self._data['path']:

                print 'Pose #{}'.format(count)
                count += 1

                pose = self._format_pose(p)

                self._move_group.set_pose_target(pose)
                self._move_group.go(wait=True)
                self._move_group.stop()
                self._move_group.clear_pose_targets()

            # if done
            if not self._repeat:
                break

    def _format_pose(self, dct):
        return Pose(
            position=Vector3(
                x=dct['position'][0],
                y=dct['position'][1],
                z=dct['position'][2]),
            orientation=Quaternion(
                x=dct['orientation'][0],
                y=dct['orientation'][1],
                z=dct['orientation'][2],
                w=dct['orientation'][3]))


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('path_reader_moveit')

    filepath = rospy.get_param('~filepath')
    repeat = rospy.get_param('~repeat',False)
    move_group = rospy.get_param('~move_group',DEFAULT_MOVE_GROUP)

    node = PathReaderMoveit(filepath, repeat, move_group)
    node.spin()
