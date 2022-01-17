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
Path Saver Node

Terminal ROS utility that listens to the tranform specified and when user
presses enter will record Pose to list. When user enters q, the
application terminates and the program saves path as a JSON file with a list
of serialized, ordered poses.

Subscribers:
    - /tf := tfMessage
        "Standard transform tree"

Parameters:
    - ~filepath
        "JSON path file save location"
    - ~base_frame (/base_link)
        "Reference transform frame, if not provided then (default)"
    - ~ee_frame (/ee_link)
        "Target transform frame, if not provided then (default)"
'''

import tf
import json
import rospy


DEFAULT_BASE_FRAME = '/base_link'
DEFAULT_EE_FRAME = '/ee_link'


class PathSaver:

    def __init__(self, filepath, base_frame, ee_frame):
        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._filepath = filepath

        self._listener = tf.TransformListener()

    def spin(self):
        data = {
            'base_frame': self._base_frame,
            'ee_frame': self._ee_frame,
            'path': []
        }

        file = open(self._filepath,'w')

        try:
            while not rospy.is_shutdown():
                inStr = raw_input('Press enter to capture pose (or press q to quit)')
                if inStr.lower() == 'q':
                    break
                (pos, rot) = self._listener.lookupTransform(self._base_frame, self._ee_frame, rospy.Time(0))
                data['path'].append({'position': pos, 'orientation': rot})
        except:
            pass

        json.dump(data, file, indent=4)
        file.close()


if __name__ == "__main__":
    rospy.init_node('path_saver')

    filepath = rospy.get_param('~filepath')
    base_frame = rospy.get_param('~base_frame',DEFAULT_BASE_FRAME)
    ee_frame = rospy.get_param('~ee_frame',DEFAULT_EE_FRAME)

    node = PathSaver(filepath, base_frame, ee_frame)
    node.spin()
