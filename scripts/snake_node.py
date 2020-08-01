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
from geometry_msgs.msg import Twist, PoseArray, Pose,  PointStamped
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Empty, EmptyResponse

import numpy as np
from random import random
import pygame
from threading import Lock
import math
from angles import shortest_angular_distance
from tf import transformations as tfs

def makeVector(x, y):
    """helper function to create column vectors in numpy"""
    return np.array([[x, y]], dtype=np.double).T

def getHeadingAngle(vector):
    """helper function to find heading angle from vector"""
    return math.atan2(vector[1], vector[0])

def getQuaternion(vector):
    """helper function to get the quaternion representation of a heading angle"""
    return tfs.quaternion_from_euler(0, 0, getHeadingAngle(vector))

def makeRotationMatrix(yaw):
    """function to create a 2D rotation matrix from a yaw angle"""
    return tfs.rotation_matrix(yaw, (0, 0, 1))[:2,:2]

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
        self.scaling = rospy.get_param('~rendering/scaling', 50)
        windowSize = int((bounds + 2*segmentRadius) * self.scaling)
        pygame.init()
        self.screen = pygame.display.set_mode((windowSize, windowSize))

    def toDisplayCoords(self, position):
        """convert game coordinates to display coordinates"""
        display = position + makeVector(1, 1) * self.segmentRadius
        display = np.matmul(np.array([[1, 0], [0, -1]]), display)
        display += makeVector(0, self.bounds + 2*self.segmentRadius)
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
        self.bounds = rospy.get_param('~bounds', 10)
        self.pathResolution = rospy.get_param('~snake/path_resolution', 0.01)
        self.segmentFollowDist = rospy.get_param('~snake/segment_follow_dist', 0.75)
        self.segmentRadius = rospy.get_param('~snake/segment_radius', 0.5)

        self.reset()

        self.renderEnabled = rospy.get_param('~render/enabled', True)
        if self.renderEnabled:
            self.renderer = SnakeGameRenderer(self.bounds, self.segmentRadius)
            self.render()

    def reset(self):
        """reset the game"""
        self.active = True
        self.lastUpdateTime = None
        self.segments = 3
        self.generatePostion()
        self.generateGoal()
        pass

    def generatePostion(self):
        """the initial starting position / path"""
        # TODO randomize?
        self.headingVector = makeVector(0, 1)
        x = 6
        y_max = 8
        y_min = y_max - (self.segments-1) * self.segmentFollowDist
        self.position = [makeVector(x, y) for y in np.linspace(y_max, y_min, self.segments)]
        # numpy.arange isn't recommended for non integer step sizes, so use linspace instead
        self.path = [makeVector(x, y) for y in np.linspace(y_max, y_min, (y_max - y_min)/self.pathResolution)]

    def step(self, (linearVelocity, angularVelocity)):
        """advance one time-step in the game"""
        if not self.lastUpdateTime:
            self.lastUpdateTime = rospy.Time.now()

        if self.active:
            # update time
            now = rospy.Time.now()
            deltaT = (now - self.lastUpdateTime).to_sec()
            self.lastUpdateTime = now

            # update heading
            rotationMatrix = makeRotationMatrix(angularVelocity*deltaT)
            headingVector = np.matmul(rotationMatrix, self.headingVector)

            # limit angle to +/- 45 degrees
            bodyVector = tfs.unit_vector(self.position[0] - self.position[1])
            angle = shortest_angular_distance(getHeadingAngle(bodyVector), getHeadingAngle(headingVector))
            if angle > math.pi/4:
                rotationMatrix = makeRotationMatrix(math.pi/4)
                headingVector = np.matmul(rotationMatrix, bodyVector)
            elif angle < -math.pi/4:
                rotationMatrix = makeRotationMatrix(-math.pi/4)
                headingVector = np.matmul(rotationMatrix, bodyVector)

            # update head position
            if linearVelocity < 0:
                rospy.logwarn('ignoring negative linear velocity command')
                linearVelocity = 0
            headPosition = self.position[0] + linearVelocity * deltaT * headingVector

            outOfBounds = np.count_nonzero(np.logical_and(headPosition >= 0, headPosition < self.bounds)) != 2
            selfIntersect = self.getDistToSelf(headPosition, startIndex=2) < self.segmentFollowDist
            if outOfBounds or selfIntersect:
                self.active = False
                rospy.loginfo('snake died due to ' +
                    ('traveling out of bounds' if outOfBounds else '') +
                    ('self intersection' if selfIntersect else ''))
            else:
                # update path
                if tfs.vector_norm(headPosition - self.path[0]) >= self.pathResolution:
                    self.path.insert(0, np.copy(headPosition))

                # update head position
                self.position[0] = headPosition
                self.headingVector = headingVector

                # Update tail
                segmentIndex = 1
                self.position = self.position[:segmentIndex]
                for pathIndex, position in enumerate(self.path):
                    if tfs.vector_norm(self.position[segmentIndex-1] - position) >= self.segmentFollowDist:
                        self.position.append(np.copy(position))
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
            goal = makeVector(random(), random()) * self.bounds
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

class SnakeGameROS:
    """ROS wrapper for the snake game"""
    def __init__(self):
        """constructor"""
        self.game = SnakeGame()
        self.lock = Lock()
        self.lastCommand = (0.0, 0.0)
        self.lastCommandTime = rospy.Time.now()

        self.frame_id = rospy.get_param('~frame_id', 'game')
        self.timeout = rospy.get_param('~timeout', 1.0)
        rate = rospy.Rate(rospy.get_param('~rate', 30)) #Hz

        # Publishers
        self.posePub = rospy.Publisher('snake/pose', PoseArray, queue_size=3)
        self.goalPub = rospy.Publisher('snake/goal', PointStamped, queue_size=3)
        self.scorePub = rospy.Publisher('snake/score', Int32, queue_size=3)
        self.activePub = rospy.Publisher('snake/active', Bool, queue_size=3)

        # Subscribers
        commandSub = rospy.Subscriber('snake/cmd_vel', Twist, self.commandCallback)

        # Services
        resetSrv = rospy.Service('snake/reset', Empty, self.resetCallback)

        try:
            while not rospy.is_shutdown():
                self.loopOnce()
                rate.sleep()
        except rospy.ROSInterruptException:
            # catch exception thrown when ROS is shutdown during sleep
            pass

    def commandCallback(self, commandMsg):
        """callback for command messages for snake"""
        self.lock.acquire()
        self.lastCommandTime = rospy.Time.now()
        self.lastCommand = (commandMsg.linear.x, commandMsg.angular.z)
        self.lock.release()

    def resetCallback(self, __):
        """callback for game reset service"""
        self.lock.acquire()
        self.game.reset()
        self.lastCommand = (0.0, 0.0)
        self.lock.release()
        return EmptyResponse()

    def loopOnce(self):
        """main loop"""
        self.lock.acquire()
        now = rospy.Time.now()

        # iterate game one step
        if (now - self.lastCommandTime).to_sec() >= self.timeout:
            self.game.step((0.0, 0.0))
        else:
            self.game.step(self.lastCommand)

        # send status messages
        poseMsg = PoseArray()
        poseMsg.header.stamp = now
        poseMsg.header.frame_id = self.frame_id
        poseMsg.poses = []
        for index, position in enumerate(self.game.position):
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            quat = None
            if index == 0:
                quat = getQuaternion(self.game.headingVector)
            else:
                quat = getQuaternion(self.game.position[index-1] - position)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            poseMsg.poses.append(pose)
        self.posePub.publish(poseMsg)

        if not self.game.goalPosition is None:
            goalMsg = PointStamped()
            goalMsg.header.stamp = now
            goalMsg.header.frame_id = self.frame_id
            goalMsg.point.x, goalMsg.point.y = self.game.goalPosition
            self.goalPub.publish(goalMsg)

        self.scorePub.publish(self.game.segments)
        self.activePub.publish(self.game.active)
        self.lock.release()

if __name__ == "__main__":
    rospy.init_node('snake_node', anonymous=False)
    snakeGameROS = SnakeGameROS()
