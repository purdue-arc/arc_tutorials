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

import rospy
from geometry_msgs.msg import Twist, PoseArray, Pose, Point, PointStamped
from std_msgs.msg import Int32, Bool

from enum import Enum
import numpy as np
from random import sample
# import pyglet
from threading import Lock

class Command(Enum):
    """Enum to handle commands for snake"""
    LEFT = 1
    FORWARD = 2
    RIGHT = 3

class SnakeGame:
    """A simple game of Snake with ROS bindings"""
    def __init__(self):
        """constructor"""
        self.renderEnabled = True
        self.size = 10
        self.score = 0
        self.active = True
        # TODO randomly initialize starting position
        self.position = [(8,6), (8,5), (8,4)]
        self.generateGoal()

    def step(self, nextCommand):
        """advance one time-step in the game"""
        if self.active:
            # Find new head position
            forwardVector = np.array(self.position[0]) - np.array(self.position[1])
            if nextCommand == Command.LEFT:
                forwardVector = np.multiply(np.array([[0, -1], [1, 0]]), forwardVector)
            elif nextCommand == Command.RIGHT:
                forwardVector = np.multiply(np.array([[0, 1], [-1, 0]]), forwardVector)
            elif nextCommand == Command.FORWARD:
                pass
            else:
                raise Exception("Invalid command")
            headPosition = np.array(self.position[0]) + forwardVector

            # Check bounds
            if np.count_nonzero(np.logical_and(headPosition >= 0, headPosition < self.size)) != 2:
                self.active = False
            else:
                # update head position
                headPosition = (headPosition[0], headPosition[1])
                self.position.insert(0, headPosition)

                # Check goal and move tail (if needed)
                if headPosition == self.goalPosition:
                    self.score += 1
                    generateGoal()
                else:
                    # remove the very last tail position because it has moved
                    # also hasn't grown from reaching goal
                    self.position.pop()

        if self.renderEnabled:
            self.render()

    def generateGoal(self):
        """generate a goal position that isn't occupied"""
        # use a linear index to sample
        total_spaces = range(0, pow(self.size, 2))
        occupied_spaces = [x * self.size + y for x, y in self.position]
        free_spaces = [space for space in total_spaces if space not in occupied_spaces]

        goal = sample(free_spaces, 1)[0]
        self.goalPosition = (goal // self.size, goal % self.size)

    def render(self):
        """render the current state of the game"""
        # TODO use pyglet to render
        pass

class ThreadedCommand:
    def __init__(self):
        self.lock = Lock()
        self.command = Command.FORWARD

def commandCb(commandMsg, threadedCommand):
    """callback for command messages for snake"""
    threadedCommand.lock.acquire()
    if commandMsg.angular.z > 0:
        threadedCommand.command = Command.LEFT
    elif commandMsg.angular.z < 0:
        threadedCommand.command = Command.RIGHT
    else:
        threadedCommand.command = Command.FORWARD
    threadedCommand.lock.release()

if __name__ == "__main__":
    snakeGame = SnakeGame()
    threadedCommand = ThreadedCommand()

    rospy.init_node('snake_node', anonymous=False)
    posePub = rospy.Publisher('pose', PoseArray, queue_size=3)
    goalPub = rospy.Publisher('goal', PointStamped, queue_size=3)
    scorePub = rospy.Publisher('score', Int32, queue_size=3)
    activePub = rospy.Publisher('active', Bool, queue_size=3)
    commandSub = rospy.Subscriber('command', Twist, commandCb, threadedCommand)

    # snakeGame.renderEnabled = rospy.get_param('~/renderEnabled', False)
    frame_id = rospy.get_param('~/frame_id', 'game')

    # TODO: wait for subscribers to connect
    # or make step require recieving a command

    # main loop
    try:
        rate = rospy.Rate(rospy.get_param('~/rate', 1)) # 10 Hz
        while not rospy.is_shutdown():
            timestamp = rospy.Time.now()

            threadedCommand.lock.acquire()
            snakeGame.step(threadedCommand.command)
            threadedCommand.command = Command.FORWARD
            threadedCommand.lock.release()
            print [(x,y) for x, y in snakeGame.position]

            poseMsg = PoseArray()
            poseMsg.header.stamp = timestamp
            poseMsg.header.frame_id = frame_id
            poseMsg.poses = [Pose(position=Point(x=x, y=y)) for x, y in snakeGame.position]
            posePub.publish(poseMsg)

            goalMsg = PointStamped()
            goalMsg.header.stamp = timestamp
            goalMsg.header.frame_id = frame_id
            goalMsg.point.x, goalMsg.point.y = snakeGame.goalPosition
            goalPub.publish(goalMsg)

            scorePub.publish(snakeGame.score)
            activePub.publish(snakeGame.active)

            rate.sleep()
    except rospy.ROSInterruptException:
        # catch exception thrown when ROS is shutdown during sleep
        pass
