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

import pygame
from vector import vector

class renderer(Object):
    """a helper class to render the Snake game in pygame"""
    GRAY = (200, 200, 200)
    RED = (255, 0, 0)
    GREEN = (0, 200, 0)
    YELLOW = (150, 200, 0)

    class ShutdownError(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, bounds, padding, scaling):
        self.bounds = bounds
        self.padding = padding
        self.scaling = scaling
        window_size = int(scaling * (bounds + 2*padding))
        pygame.init()
        self._screen = pygame.display.set_mode((window_size, window_size))

    def convert_to_display_coords(self, position):
        """convert game coordinates to display coordinates"""
        display = position + makeVector(1, 1) * self.segmentRadius
        display = np.matmul(np.array([[1, 0], [0, -1]]), display)
        display += makeVector(0, self.bounds + 2*self.segmentRadius)
        display *= self.scaling
        return display.astype(np.int32)

    def render(self, goal, segments):
        """render the current state of the game"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()
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
