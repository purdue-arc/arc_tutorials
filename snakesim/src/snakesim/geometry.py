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

import numpy as np
from tf import transformations as tfs

class rotation_matrix(np.ndarray):
    """A 2D rotation matrix"""
    def __new__(cls, yaw):
        return tfs.rotation_matrix(yaw, (0, 0, 1))[:2,:2].view(cls)

class vector(np.ndarray):
    """A 2D vector object."""
    def __new__(cls, x, y):
        return np.array([[x, y]], dtype=np.double).T.view(cls)

    @property
    def x(self):
        """X component."""
        return self[0]

    @property
    def y(self):
        """Y component."""
        return self[1]

    @property
    def magnitude(self):
        """Scalar magnitude."""
        return tfs.vector_norm(self)

    @property
    def unit(self):
        """Unit vector."""
        return tfs.unit_vector(self).view(vector)
