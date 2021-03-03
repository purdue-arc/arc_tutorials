"""Contains the Game class.

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

from snakesim.arena import Arena
from snakesim.goal import Goal
from snakesim.renderer import Renderer
from snakesim.snake import Snake
from snakesim.geometry import Vector

import random

class Game(object):
    """A simple game of Snake."""
    def __init__(self, bounds=10, segment_radius=0.5, segment_follow_dist=0.75,
                 path_resolution=0.01, render=True, render_scaling=50,
                 initial_segments=3, snake_growth=1):
        self.arena = Arena(bounds, bounds)
        self.goal = Goal(self.arena, radius=segment_radius)
        snake_len = segment_follow_dist*(initial_segments-1) + 2*segment_radius
        position, heading = self.get_random_start(bounds, 0.8, snake_len, segment_radius)
        self.snake = Snake(position, heading,
                           num_segments=initial_segments,
                           growth=snake_growth,
                           radius=segment_radius,
                           follow_distance=segment_follow_dist,
                           path_resolution=path_resolution)
        self.goal.randomize(self.snake)
        self.active = True
        self.score = 0

        self.render_enabled = render
        if self.render_enabled:
            self.renderer = Renderer(bounds, segment_radius, scaling=render_scaling)
            self._render()

    def get_random_start(self, bounds, usable, snake_len, radius):
        """Generate random starting position and heading."""
        heading = Vector(random.uniform(-1.0, 1.0),
                        random.uniform(-1.0, 1.0)).unit()
        mini_bounds = bounds*usable
        Xmax = mini_bounds - snake_len*abs(heading.x)
        Ymax = mini_bounds - snake_len*abs(heading.y)

        if heading.x > 0:
            X0 = random.uniform(0, Xmax)
        else:
            X0 = random.uniform(Xmax, mini_bounds)

        if heading.y > 0:
            Y0 = random.uniform(0, Ymax)
        else:
            Y0 = random.uniform(Ymax, mini_bounds)

        X0 += (bounds-mini_bounds)/2
        Y0 += (bounds-mini_bounds)/2

        tail = Vector(X0, Y0)
        head = tail + (snake_len-radius)*heading

        return (head, heading)

    def reset(self):
        """reset the game"""
        copy = self.snake
        snake_len = copy.follow_distance*(copy.initial_segments-1) + 2*copy.head.radius
        position, heading = self.get_random_start(self.arena.width, 0.8, snake_len, copy.head.radius)
        self.snake = Snake(position, heading,
                            num_segments=copy.initial_segments,
                            growth=copy.growth,
                            radius=copy.head.radius,
                            follow_distance=copy.follow_distance,
                            path_resolution=copy.path_resolution)
        self.goal.randomize(self.snake)
        self.active = True
        self.score = 0

    def step(self, linear_velocity, angular_velocity, delta_t):
        """Advance one time-step in the game."""
        if self.active:
            try:
                self.snake.step(linear_velocity, angular_velocity, delta_t)
            except Snake.SelfIntersectionError:
                self.active = False

            if not self.arena.check_position(self.snake.head.position):
                self.active = False

            if (self.active
                    and self.goal.position is not None
                    and (self.goal.position - self.snake.head.position).magnitude()
                    <= self.goal.radius):
                self.score += 1
                self.snake.grow()
                self.goal.randomize(self.snake)

        if self.render_enabled:
            self._render()

    def _render(self):
        """Render the current state of the game."""
        try:
            self.renderer.render(self.goal, self.snake)
        except Renderer.ShutdownError:
            self.render_enabled = False
