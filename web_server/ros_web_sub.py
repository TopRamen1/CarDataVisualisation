#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import tf
import math
import rospy
import json
import websocket
import sys, traceback
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion

ws = websocket.WebSocket()
ws.connect("ws://192.168.10.30:8000/ws/robot/guy/")

def callback(data):
    global ws
    pose = listener.lookupTransform('/map', '/back_wheels', rospy.Time(0.0))
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(pose[1])
    position = {
        'x': pose[0][0],
        'y': pose[0][1],
        'theta': yaw * 180.0/math.pi
    }
    message = json.dumps({ 'message' : position })
    ws.send('%s' % message)




if __name__ == '__main__':
    rospy.init_node('robotDispatcher')
    listener = tf.TransformListener()
    try:
        rospy.Subscriber("/dv_path_planning/path", PoseArray, callback)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

        traceback.print_exc(file=sys.stdout)


    rospy.spin()
