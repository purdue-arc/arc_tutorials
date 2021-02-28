"""Contains the Snake class.

License:
  BSD 3-Clause License

  Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import math
import numpy as np

from snakesim.segment import Segment

class Snake(object):
    """A snake."""
    MAX_ANGLE = math.pi/4

    class SelfIntersectionError(Exception):
        """Exception for when the snake self intersects"""
        pass

    def __init__(self, position, heading_vector, num_segments=3, growth=1,
                 radius=0.5, follow_distance=0.75, path_resolution=0.01):
        self.initial_segments = num_segments
        self.growth = growth
        self.follow_distance = follow_distance
        self.path_resolution = path_resolution

        self.segments = []
        for segment_num in range(num_segments):
            segment_position = position - segment_num*heading_vector*follow_distance
            segment = Segment(radius, segment_position, heading_vector)
            self.segments.append(segment)

        length = (num_segments-1) * follow_distance
        count = length/path_resolution + 1
        self._path = [position - delta*heading_vector for delta in np.linspace(0, length, count)]

    @property
    def head(self):
        """The head segment."""
        return self.segments[0]

    @property
    def body(self):
        """Array of body segments."""
        return self.segments[1:]

    def step(self, linear_velocity, angular_velocity, delta_t):
        """Advance one time step"""
        # move head
        delta_yaw = angular_velocity * delta_t
        heading_vector = self.head.heading_vector.rotate(delta_yaw)
        body_vector = (self.head.position - self.body[0].position).unit()

        dp = np.dot(body_vector, heading_vector)
        dp = max(min(dp, 1.0), -1.0)

        if math.acos(dp) > self.MAX_ANGLE:
            if np.cross(body_vector, heading_vector) > 0:
                heading_vector = body_vector.rotate(self.MAX_ANGLE)
            else:
                heading_vector = body_vector.rotate(-self.MAX_ANGLE)

        if linear_velocity < 0:
            return
        position = self.head.position + heading_vector*linear_velocity*delta_t

        self.head.position = position
        self.head.heading_vector = heading_vector

        # update body segments and trim path
        path_index = 0
        for index, segment in enumerate(self.body):
            path_index += self._update_segment(segment, self.segments[index],
                                               self._path[path_index:])
        self._path = self._path[:path_index]

        # check self intersection
        for segment in self.body:
            if (self.head.position - segment.position).magnitude() < self.follow_distance:
                raise self.SelfIntersectionError()

        # extend path
        if (self.head.position - self._path[0]).magnitude() >= self.path_resolution:
            self._path.insert(0, self.head.position.copy())

    def _update_segment(self, segment, leading_segment, path):
        """Update the position of this segment."""
        for index, position in enumerate(path):
            if (leading_segment.position - position).magnitude() >= self.follow_distance:
                segment.position = position.copy()
                segment.heading_vector = (leading_segment.position - position).unit()
                return index
        return len(path)

    def grow(self):
        """Add a new segment on to the tail."""
        for __ in range(self.growth):
            copy = self.segments[-1]
            segment = Segment(copy.radius, copy.position.copy(),
                              copy.heading_vector.copy())
            self.segments.append(segment)
