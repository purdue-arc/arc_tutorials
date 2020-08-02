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
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse

# Python
import math
from threading import Lock

def getAbsDistance(point1, point2):
    """calculate Euclidian distance between two points"""
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    return math.sqrt(delta_x**2 + delta_y**2)

class SnakePositionController:
    """Simple position controller for the snake"""
    def __init__(self):
        """constructor"""
        rospy.init_node('snake_position_controller', anonymous=False)

        self.poseQueue = []
        self.lock = Lock()

        self.poseTolerance = rospy.get_param('~poseTolerance', 0.5)

        # Publishers
        self.headingPub = rospy.Publisher('controller/heading', Float64, queue_size=1)
        self.poseQueuePub = rospy.Publisher('controller/pose/queue', PoseArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.snakeCallback)
        rospy.Subscriber('controller/pose', Pose, self.goalCallback)

        # Services
        rospy.Service('controller/pose/reset', Empty, self.resetCallback)

        rospy.spin()

    def goalCallback(self, poseMsg):
        """callback for pose goal"""
        self.lock.acquire()
        goal = (poseMsg.position.x, poseMsg.position.y)
        if not self.poseQueue:
            self.poseQueue.append(goal)
            self.publishPoseQueue()
        elif getAbsDistance(goal, self.poseQueue[-1]) > self.poseTolerance:
            self.poseQueue.append(goal)
            self.publishPoseQueue()
        else:
            rospy.loginfo_throttle(5.0, 'ignoring new goal that is too close to preceding goal')
        self.lock.release()

    def snakeCallback(self, poseMsg):
        """callback for poses from the snake"""
        self.lock.acquire()
        if self.poseQueue:
            pose = (poseMsg.poses[0].position.x, poseMsg.poses[0].position.y)
            goal = self.poseQueue[0]

            heading = math.atan2(goal[0] - pose[0], goal[1] - pose[1])
            self.headingPub.publish(heading)

            if getAbsDistance(goal, pose) < self.poseTolerance:
                self.poseQueue.pop(0)
                self.publishPoseQueue()
        self.lock.release()

    def resetCallback(self, __):
        """reset the current pose callback queue"""
        self.lock.acquire()
        self.poseQueue = []
        self.publishPoseQueue()
        self.lock.release()
        return EmptyResponse()

    def publishPoseQueue(self):
        """publish the current state of the pose queue"""
        poseArrayMsg = PoseArray()
        poseArrayMsg.poses = []
        for pose in self.poseQueue:
            poseMsg = Pose()
            poseMsg.position.x, poseMsg.position.y = pose
            poseArrayMsg.poses.append(poseMsg)
        self.poseQueuePub.publish(poseArrayMsg)

if __name__ == "__main__":
    controller = SnakePositionController()
