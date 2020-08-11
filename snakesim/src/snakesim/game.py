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

from arena import Arena
from goal import Goal
from renderer import Renderer
from snake import Snake
from geometry import Vector

class Game(object):
    """A simple game of Snake."""
    def __init__(self, bounds=10, segment_radius=0.5, segment_follow_dist=0.75,
                 path_resolution=0.01, render=True, render_scaling=50):

        self.arena = Arena(bounds, bounds)
        self.goal = Goal(self.arena, radius=segment_radius)
        self.snake = Snake(Vector(8,6), Vector(0,1), num_segments=3,
                           radius=segment_radius,
                           follow_distance=segment_follow_dist,
                           path_resolution=path_resolution)

        self.goal.randomize(self.snake)
        self.active = True

        self.render_enabled = render
        if self.render_enabled:
            self.renderer = Renderer(bounds, segment_radius, scaling=render_scaling)
            self._render()

    def reset(self):
        """reset the game"""
        copy = self.snake
        self.snake = Snake(Vector(8,6), Vector(0,1), num_segments=3,
                           radius=copy.radius,
                           follow_distance=copy.follow_distance,
                           path_resolution=copy.path_resolution)
        self.goal.randomize(self.snake)
        self.active = True

    def step(self, linear_velocity, angular_velocity, delta_t):
        """Advance one time-step in the game."""
        if self.active:
            try:
                self.snake.step(linear_velocity, angular_velocity, delta_t)
            except Snake.SelfIntersectionError:
                self.active = False

            if not self.arena.check_position(self.snake.head.position):
                self.active = False

            if (self.active and self.goal.position is not None
              and (self.goal.position - self.snake.head.position).magnitude()
              <= self.goal.radius):
                self.snake.add_segment()
                self.goal.randomize(self.snake)

        if self.render_enabled:
            self._render()

    def _render(self):
        """Render the current state of the game."""
        try:
            self.renderer.render(self.goal, self.snake)
        except Renderer.ShutdownError:
            self.render_enabled = False
