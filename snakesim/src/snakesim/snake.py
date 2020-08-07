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

    def step(self, linearVelocity, angularVelocity):
        """advance one time-step in the game"""
        if not self.lastUpdateTime:
            self.lastUpdateTime = rospy.Time.now()

        if self.active:
            # update time
            now = rospy.Time.now()
            deltaT = (now - self.lastUpdateTime).to_sec()
            self.lastUpdateTime = now

            # update heading
            

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


                # place any waiting-to-spawn segments at the end
                if not segmentIndex >= self.segments:
                    self.position.extend([self.path[-1]] * (self.segments - segmentIndex))

                # Check goal
                if not self.goalPosition is None and tfs.vector_norm(headPosition - self.goalPosition) <= self.segmentRadius:
                    self.segments += 1
                    self.generateGoal()

        if self.renderEnabled:
            self.render()



    def getDistToSelf(self, position, startIndex=0):
        """get the minimum distance of a position to any segment"""
        return min([tfs.vector_norm(segment - position) for segment in self.position[startIndex:]])

    def render(self):
        """render the current state of the game using a SnakeGameRenderer"""
        try:
            self.renderer.render(self.goalPosition, self.position)
        except(SnakeGameRenderer.ShutdownException):
            self.renderEnabled = False
