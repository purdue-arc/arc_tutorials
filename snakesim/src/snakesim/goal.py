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

class goal(Object):
    """A goal for the snake to chase"""

    MAX_ATTEMPTS = 10
    REQUIRED_SEPARATION = 0.5

    def __init__(self, arena, radius=0.5):
        self.arena = arena
        self.radius = radius
        self.position = None

    def randomize_position(self, snake):
        """Generate a goal position that isn't occupied."""
        self.position = None
        required_distance = snake.segment_radius
                            + self.radius
                            + self.REQUIRED_SEPARATION
        for __ in range(MAX_ATTEMPTS):
            goal = arena.generate_position()
            if snake.minimum_distance(goal) >= required_distance:
                self.goalPosition = goal
                return

    def check_collision(self, snake):
        """Check if the snake has met its goal."""
        return (
            self.position is not None
            and snake.minimum_distance(self.position) < snake.segment_radius
        )
