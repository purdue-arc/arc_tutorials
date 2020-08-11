#!/usr/bin/env python

################################################################################
# BSD 3-Clause License
#
# Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64

# Python
import math
from angles import shortest_angular_distance
from tf import transformations as tfs


class SnakeHeadingController:
    """Simple heading controller for the snake"""
    def __init__(self):
        """constructor"""
        rospy.init_node('snake_heading_controller', anonymous=False)

        self.headingCommand = None
        self.linearVelocityCommand = None
        self.angularVelocity = rospy.get_param('~angularVelocity', 6.28)

        # Publishers
        self.twistPub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.poseCallback)
        rospy.Subscriber('controller/heading', Float64, self.headingCallback)
        rospy.Subscriber('controller/velocity', Float64, self.velocityCallback)

        rospy.spin()

    def headingCallback(self, headingMsg):
        """callback for heading goal"""
        # This operation is atomic, so we don't need a lock
        self.headingCommand = headingMsg.data

    def velocityCallback(self, velocityMsg):
        """callback for velocity command"""
        # This operation is atomic, so we don't need a lock
        self.linearVelocityCommand = velocityMsg.data

    def poseCallback(self, poseMsg):
        """callback for poses from the snake"""
        if self.headingCommand and self.linearVelocityCommand:
            quat = (poseMsg.poses[0].orientation.x,
                    poseMsg.poses[0].orientation.y,
                    poseMsg.poses[0].orientation.z,
                    poseMsg.poses[0].orientation.w)

            __, __, heading = tfs.euler_from_quaternion(quat)
            headingError = shortest_angular_distance(heading, self.headingCommand)
            angularVelocity = math.copysign(self.angularVelocity, headingError)

            twistMsg = Twist()
            twistMsg.linear.x = self.linearVelocityCommand
            twistMsg.angular.z = angularVelocity
            self.twistPub.publish(twistMsg)

if __name__ == "__main__":
    controller = SnakeHeadingController()
