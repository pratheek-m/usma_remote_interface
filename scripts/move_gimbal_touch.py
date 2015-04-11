#!/usr/bin/env python

# Copyright (c) 2008, Willow Garage, Inc.  All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.  * Redistributions
#     in binary form must reproduce the above copyright notice, this list of
#     conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution. # Neither the name of
#     the Willow Garage, Inc. nor the names of its contributors may be used to
#     endorse or promote products derived from this software without specific
#     prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Daniel Hewlett
# Author: Antons Rebguns

PKG = 'aribo'

import roslib; roslib.load_manifest(PKG)

import time
from math import pi
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from dynamixel_controllers.srv import *
from dynamixel_msgs.msg import JointState as JointState
from geometry_msgs.msg import Twist

class MoveGimbal():
    def __init__(self):
        self.is_running = True
        self.step_size = 0.001
        self.touch_data = None
        self.prev_time = time.time()
        
        rospy.init_node('move_gimbal_touch', anonymous=True)
        rospy.Subscriber('/teleopCam', Twist, self.read_touch_data)

        self.servo_position_pan = rospy.Publisher('/pan_controller/command', Float64)
        self.servo_position_tilt = rospy.Publisher('/tilt_controller/command', Float64)

        self.pan_joint = 0.0
	self.tilt_joint = 0.0

    def read_touch_data(self, data):
        self.touch_data = data
        cur_time = time.time()
        timediff = cur_time - self.prev_time
        self.prev_time = cur_time

    def update_gimbal_position(self):
        while self.is_running:
            if self.touch_data:
		#if self.pan_joint > -2.6 and self.pan_joint < 2.6:
		#if self.pan_joint > -1.0 and self.pan_joint < 1.0:
		self.pan_joint += 1 * self.touch_data.angular.z * self.step_size
		#if self.tilt_joint > -1.7 and self.pan_joint < 1.7:
		#if self.tilt_joint > -1.0 and self.pan_joint < 1.0:
		self.tilt_joint += 1 * self.touch_data.angular.x * self.step_size
		print "touch"
	    else:
		print "no touch"
		self.pan_joint = 0.0
		self.tilt_joint = 0.0

	    self.servo_position_pan.publish(self.pan_joint)
	    self.servo_position_tilt.publish(self.tilt_joint)
            time.sleep(0.05)

if __name__ == '__main__':
    move_gimbal = MoveGimbal()
    t = Thread(target=move_gimbal.update_gimbal_position)
    t.start()
    rospy.spin()
    move_gimbal.alive = False
    t.join()


