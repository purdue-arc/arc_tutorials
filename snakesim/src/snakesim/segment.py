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

import math
from tf import tfs as tfs
import geometry

class NegativeVelocityError(Exception):
    """Exception for when the linear velocity is negative"""
    pass

class segment(Object):
    """A single segment of the snake."""
    def __init__(self, radius):
        self.radius = radius
        self.position = None
        self.heading_vector = None

    @property
    def heading_angle(self):
        """Heading of the snake as a yaw angle."""
        if self.heading_vector is not None:
            return math.atan2(self.heading_vector.y, self.heading_vector.x)
        else:
            return None

    @property
    def heading_quaternion(self):
        """Heading of the segment as a quaternion."""
        if self.heading_vector is not None:
            return tfs.quaternion_from_euler(0, 0, self.heading_angle)
        else:
            return None

class body_segment(segment):
    """A segment other than the head."""
    def __init__(self, radius, follow_distance):
        super.__init__(radius)
        self.follow_distance = follow_distance

    def follow(self, leading_segment, path):
        """Update the position of this segment."""
        for index, position in enumerate(path):
            if tfs.vector_norm()


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

class head_segment(segment):
    """The head segment of the snake."""
    MAX_ANGLE = math.pi/4

    def integrate(self, linear_velocity, angular_velocity,
                  delta_t, following_segment):
        """Advance one time step."""
        rotation_matrix = geometry.rotation_matrix(angular_velocity * delta_t)
        heading_vector = np.matmul(rotation_matrix, self.heading_vector)

        body_vector = tfs.unit_vector(self.position - following_segment.position)
        angle = math.acos(np.dot(body_vector, heading_vector))
        if angle > MAX_ANGLE:
            rotation_matrix = makeRotationMatrix(MAX_ANGLE)
            heading_vector = np.matmul(rotation_matrix, body_vector)
        elif angle < -MAX_ANGLE:
            rotation_matrix = makeRotationMatrix(-MAX_ANGLE)
            heading_vector = np.matmul(rotation_matrix, body_vector)

        if linearVelocity < 0:
            raise NegativeVelocityError()
        position = self.position + heading_vector*linearVelocity*deltaT

        if not self.arena.check_position(self.position)
            raise Error()

        self.position = position
        self.heading_vector = heading_vector
