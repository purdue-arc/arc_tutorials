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
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float64

# Python
import math

class SnakePositionController:
    """Simple position controller for the snake"""
    def __init__(self):
        """constructor"""
        rospy.init_node('snake_position_controller', anonymous=False)

        self.positionCommand = []

        # Publishers
        self.headingPub = rospy.Publisher('controller/heading', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.poseCallback)
        rospy.Subscriber('controller/position', Point, self.positionCallback)

        rospy.spin()

    def positionCallback(self, positionMsg):
        """callback for position goal"""
        # This operation is atomic, so we don't need a lock
        self.positionCommand.append((positionMsg.x, positionMsg.y))
        print self.positionCommand

    def poseCallback(self, poseMsg):
        """callback for poses from the snake"""
        if self.positionCommand:
            command_x, command_y = self.positionCommand[0]
            error_x = command_x - poseMsg.poses[0].position.x
            error_y = command_y - poseMsg.poses[0].position.y

            heading = math.atan2(error_y, error_x)
            self.headingPub.publish(heading)

            if math.sqrt(error_x**2 + error_y**2) < 0.5:
                self.positionCommand.pop(0)
                print self.positionCommand

if __name__ == "__main__":
    controller = SnakePositionController()
