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

import numpy as np
from random import sample
import pygame
from threading import Lock
from math import sqrt, ceil

def dist(vector):
    """calculate the euclidian distance of a numpy vector"""
    return sqrt(np.sum(np.square(vector)))

class SnakeGameRenderer:
    """a helper class to render the Snake game in pygame"""
    GRAY = (200, 200, 200)
    RED = (255, 0, 0)
    GREEN = (0, 200, 0)
    YELLOW = (150, 200, 0)

    def __init__(self, game):
        self.game = game
        self.scaling = rospy.get_param('~/rendering/scaling', 100)
        windowSize = int((game.bounds + 2*game.segmentRadius) * self.scaling)
        pygame.init()
        self.screen = pygame.display.set_mode((windowSize, windowSize))
        self.render()

    def toDisplayCoords(self, position):
        """convert game coordinates to display coordinates"""
        display = position + np.array([[1, 1]]).T*self.game.segmentRadius
        display = np.matmul(np.array([[1, 0], [0, -1]]), display)
        display += np.array([[0, self.game.bounds + 2*self.game.segmentRadius]]).T
        display *= self.scaling
        return display.astype(np.int32)

    def render(self):
        """render the current state of the game"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                game.renderEnabled = False
                pygame.quit()
                return
        self.screen.fill(self.GRAY)
        radius = int(self.game.segmentRadius * self.scaling)
        # Draw the goal
        pygame.draw.circle(self.screen, self.RED, self.toDisplayCoords(self.game.goalPosition), radius)
        # Draw segments
        for position in self.game.position[1:]:
            pygame.draw.circle(self.screen, self.GREEN, self.toDisplayCoords(position), radius)
        # Draw the head in a different color
        pygame.draw.circle(self.screen, self.YELLOW, self.toDisplayCoords(self.game.position[0]), radius)
        pygame.display.flip()

class SnakeGame:
    """A simple game of Snake with ROS bindings"""
    def __init__(self):
        """constructor"""
        self.active = True
        self.lastUpdateTime = None
        self.bounds = 10

        # TODO randomly initialize starting position
        self.heading = np.array([[0, 1]], dtype=np.double).T
        self.segments = 3
        self.segmentFollowDist = 0.75
        self.position = [np.array([[8, 6 - self.segmentFollowDist * y]], dtype=np.double).T for y in range(self.segments)]

        self.pathResolution = 0.01
        self.path = [np.array([[8, 6 - self.pathResolution * y]], dtype=np.double).T for y in range(1+int(ceil((self.segments-1) * self.segmentFollowDist / self.pathResolution)))]
        self.segmentRadius = 0.5

        self.generateGoal()

        self.renderEnabled = rospy.get_param('~/rendering/enabled', True)
        if self.renderEnabled:
            self.renderer = SnakeGameRenderer(self)
            self.renderer.render()

    def step(self, nextCommand):
        """advance one time-step in the game"""
        if not self.lastUpdateTime:
            self.lastUpdateTime = rospy.Time.now()

        if self.active:
            # update time
            now = rospy.Time.now()
            deltaT = (now - self.lastUpdateTime).to_sec()
            self.lastUpdateTime = now

            # update head position
            headPosition = self.position[0] + nextCommand[0] * deltaT * self.heading

            # update heading
            sinTheta = np.sin(nextCommand[1] * deltaT)
            cosTheta = np.cos(nextCommand[1] * deltaT)
            rotationMatrix = np.array([[cosTheta, -sinTheta], [sinTheta, cosTheta]])
            heading = np.matmul(rotationMatrix, self.heading)

            # Check bounds
            # TODO, didn't seem to work
            if np.count_nonzero(np.logical_and(headPosition >= 0, headPosition < self.bounds)) != 2:
                self.active = False
            else:
                # update path
                if dist(headPosition - self.position[0]) >= self.pathResolution:
                    self.path.insert(0, headPosition)

                # update head position
                self.position[0] = headPosition
                self.heading = heading

                # Update tail
                segmentIndex = 1
                self.position = self.position[:1]
                for pathIndex, position in enumerate(self.path):
                    if dist(self.position[segmentIndex-1] - position) >= self.segmentFollowDist:
                        self.position.append(position)
                        segmentIndex += 1
                        if segmentIndex >= self.segments:
                            # if we aren't waiting to spawn new segments,
                            # we don't need to track the path longer than the snake
                            self.path = self.path[:pathIndex+1]
                            break


                # place any waiting-to-spawn segments at the end
                if not segmentIndex >= self.segments:
                    self.position.extend([self.path[-1]] * (self.segments - segmentIndex))

                # Check goal
                if dist(headPosition - self.goalPosition) <= self.segmentRadius:
                    self.segments = 5
                    self.generateGoal()

        if self.renderEnabled:
            self.renderer.render()

    def generateGoal(self):
        """generate a goal position that isn't occupied"""
        # TODO random sample
        self.goalPosition = np.array([[4, 6]], dtype=np.double).T

class ThreadedCommand:
    def __init__(self):
        self.lock = Lock()
        self.command = (0, 0)
        self.lastTime = rospy.Time.now()

def commandCb(commandMsg, threadedCommand):
    """callback for command messages for snake"""
    threadedCommand.lock.acquire()
    threadedCommand.command = (commandMsg.linear.x, commandMsg.angular.z)
    threadedCommand.lastTime = rospy.Time.now()
    threadedCommand.lock.release()

if __name__ == "__main__":
    rospy.init_node('snake_node', anonymous=False)

    snakeGame = SnakeGame()
    threadedCommand = ThreadedCommand()

    posePub = rospy.Publisher('pose', PoseArray, queue_size=3)
    goalPub = rospy.Publisher('goal', PointStamped, queue_size=3)
    scorePub = rospy.Publisher('score', Int32, queue_size=3)
    activePub = rospy.Publisher('active', Bool, queue_size=3)
    commandSub = rospy.Subscriber('cmd_vel', Twist, commandCb, threadedCommand)

    frame_id = rospy.get_param('~/frame_id', 'game')

    # main loop
    try:
        rate = rospy.Rate(rospy.get_param('~/rate', 30)) # 10 Hz
        while not rospy.is_shutdown():
            timestamp = rospy.Time.now()

            threadedCommand.lock.acquire()
            snakeGame.step(threadedCommand.command)
            # TODO timeout
            threadedCommand.lock.release()

            poseMsg = PoseArray()
            poseMsg.header.stamp = timestamp
            poseMsg.header.frame_id = frame_id
            # TODO orientation
            poseMsg.poses = [Pose(position=Point(x=x, y=y)) for x, y in snakeGame.position]
            posePub.publish(poseMsg)

            goalMsg = PointStamped()
            goalMsg.header.stamp = timestamp
            goalMsg.header.frame_id = frame_id
            goalMsg.point.x, goalMsg.point.y = snakeGame.goalPosition
            goalPub.publish(goalMsg)

            scorePub.publish(snakeGame.segments)
            activePub.publish(snakeGame.active)

            rate.sleep()
    except rospy.ROSInterruptException:
        # catch exception thrown when ROS is shutdown during sleep
        pass
