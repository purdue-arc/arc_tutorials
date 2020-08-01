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
from random import random
import pygame
from threading import Lock
import math
from angles import shortest_angular_distance
from tf import transformations as tfs

def vector(x, y):
    """helper function to create column vectors in numpy"""
    return np.array([[x, y]], dtype=np.double).T

def getHeadingAngle(vector):
    """helper function to find heading angle from vector"""
    return math.atan2(vector[1], vector[0])

def getQuaternion(vector):
    """helper function to get the quaternion representation of a heading angle"""
    return tfs.quaternion_from_euler(0, 0, heading)

class SnakeGameRenderer:
    """a helper class to render the Snake game in pygame"""
    GRAY = (200, 200, 200)
    RED = (255, 0, 0)
    GREEN = (0, 200, 0)
    YELLOW = (150, 200, 0)

    class ShutdownException(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, bounds, segmentRadius):
        self.bounds = bounds
        self.segmentRadius = segmentRadius
        self.scaling = rospy.get_param('~/rendering/scaling', 50)
        windowSize = int((bounds + 2*segmentRadius) * self.scaling)
        pygame.init()
        self.screen = pygame.display.set_mode((windowSize, windowSize))

    def toDisplayCoords(self, position):
        """convert game coordinates to display coordinates"""
        display = position + vector(1, 1) * self.segmentRadius
        display = np.matmul(np.array([[1, 0], [0, -1]]), display)
        display += vector(0, self.bounds + 2*self.segmentRadius)
        display *= self.scaling
        return display.astype(np.int32)

    def render(self, goalPosition, snakePosition):
        """render the current state of the game"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise SnakeGameRenderer.ShutdownException()
        self.screen.fill(self.GRAY)
        radius = int(self.segmentRadius * self.scaling)
        # Draw the goal
        if not goalPosition is None:
            pygame.draw.circle(self.screen, self.RED, self.toDisplayCoords(goalPosition), radius)
        # Draw segments
        for position in snakePosition[1:]:
            pygame.draw.circle(self.screen, self.GREEN, self.toDisplayCoords(position), radius)
        # Draw the head in a different color
        pygame.draw.circle(self.screen, self.YELLOW, self.toDisplayCoords(snakePosition[0]), radius)
        pygame.display.flip()

class SnakeGame:
    """A simple game of Snake with ROS bindings"""
    def __init__(self):
        """constructor"""
        self.bounds = rospy.get_param('~/bounds', 10)
        self.pathResolution = rospy.get_param('~/snake/path_resolution', 0.01)
        self.segmentFollowDist = rospy.get_param('~/snake/segment_follow_dist', 0.75)
        self.segmentRadius = rospy.get_param('~/snake/segment_radius', 0.5)

        self.active = True
        self.lastUpdateTime = None
        self.segments = 3

        # TODO randomly initialize starting position
        self.headingVector = vector(0, 1)
        self.position = [vector(8, 6 - self.segmentFollowDist * y) for y in range(self.segments)]
        self.path = [vector(8, 6 - self.pathResolution * y) for y in range(1+int(math.ceil((self.segments-1) * self.segmentFollowDist / self.pathResolution)))]

        self.generateGoal()

        self.renderEnabled = rospy.get_param('~/rendering/enabled', True)
        if self.renderEnabled:
            self.renderer = SnakeGameRenderer(self.bounds, self.segmentRadius)
            self.render()

    def step(self, nextCommand):
        """advance one time-step in the game"""
        if not self.lastUpdateTime:
            self.lastUpdateTime = rospy.Time.now()

        if self.active:
            # update time
            now = rospy.Time.now()
            deltaT = (now - self.lastUpdateTime).to_sec()
            self.lastUpdateTime = now

            # update heading
            rotationMatrix = tfs.rotation_matrix(nextCommand[1]*deltaT, (0, 0, 1))[:2,:2]
            heading = np.matmul(rotationMatrix, self.headingVector)

            # limit angle to +/- 90 degrees
            angle = shortest_angular_distance(getHeadingAngle(self.position[0] - self.position[1]), getHeadingAngle(self.headingVector))
            if angle > math.pi/4:
                rotationMatrix = tfs.rotation_matrix(math.pi/4, (0, 0, 1))[:2,:2]
                perpendicularVector = self.position[0] - self.position[1]
                heading = np.matmul(rotationMatrix, perpendicularVector)
            elif angle < -math.pi/4:
                rotationMatrix = tfs.rotation_matrix(-math.pi/4, (0, 0, 1))[:2,:2]
                perpendicularVector = self.position[0] - self.position[1]
                heading = np.matmul(rotationMatrix, perpendicularVector)
            print angle

            # update head position
            headPosition = self.position[0] + nextCommand[0] * deltaT * heading

            outOfBounds = np.count_nonzero(np.logical_and(headPosition >= 0, headPosition < self.bounds)) != 2
            selfIntersect = self.getDistToSelf(headPosition, startIndex=2) < self.segmentFollowDist
            if outOfBounds or selfIntersect:
                self.active = False
            else:
                # update path
                if tfs.vector_norm(headPosition - self.position[0]) >= self.pathResolution:
                    self.path.insert(0, headPosition)

                # update head position
                self.position[0] = headPosition
                self.headingVector = heading

                # Update tail
                segmentIndex = 1
                self.position = self.position[:1]
                for pathIndex, position in enumerate(self.path):
                    if tfs.vector_norm(self.position[segmentIndex-1] - position) >= self.segmentFollowDist:
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
                if not self.goalPosition is None and tfs.vector_norm(headPosition - self.goalPosition) <= self.segmentRadius:
                    self.segments += 1
                    self.generateGoal()

        if self.renderEnabled:
            self.render()

    def generateGoal(self):
        """generate a goal position that isn't occupied"""
        self.goalPosition = None
        # arbitrary number of attempts before giving up and waiting for the next timestep
        # we don't want to hold up the loop
        for __ in range(10):
            goal = vector(random(), random()) * self.bounds
            # arbitrarily said I want a 1/2 segment gap
            if self.getDistToSelf(goal) >= 3 * self.segmentRadius:
                self.goalPosition = goal
                return

    def getDistToSelf(self, position, startIndex=0):
        """get the minimum distance of a position to any segment"""
        return min([tfs.vector_norm(segment - position) for segment in self.position[startIndex:]])

    def render(self):
        """render the current state of the game using a SnakeGameRenderer"""
        try:
            self.renderer.render(self.goalPosition, self.position)
        except(SnakeGameRenderer.ShutdownException):
            self.renderEnabled = False

class ThreadedCommand:
    def __init__(self):
        self.lock = Lock()
        self.command = (0, 0)
        self.lastTime = rospy.Time.now()

def commandCb(commandMsg, threadedCommand):
    """callback for command messages for snake"""
    threadedCommand.lock.acquire()
    threadedCommand.lastTime = rospy.Time.now()
    if commandMsg.linear.x < 0:
        rospy.logwarn('ignoring negative linear velocity command')
        commandMsg.linear.x = 0
    threadedCommand.command = (commandMsg.linear.x, commandMsg.angular.z)
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

            if not snakeGame.goalPosition is None:
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
