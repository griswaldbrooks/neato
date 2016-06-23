#!/usr/bin/env python
#
# Software Licence Agreement (MIT)
#
# Copyright (c) 2016 Griswald Brooks
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
# TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
#

##
# @author Griswald Brooks

## @file command_robot.py Module for computing drive commands for the Botvac.

import numpy as np
from numpy import linalg as la


def get_wheel_vel(v, w, r):
    """Function to get the wheel velocities for diff drive robot."""
    v_l = v - w*r
    v_r = v + w*r

    return [v_l, v_r]


def get_wheel_cmds_to_rel_pose_triple(x, y, theta):
    """Gives rotate, drive, rotate commands to a pose relative to its current pose."""
    # Compute rotation, drive, rotation. Current orientation is 0 degrees.
    robot_width = 245.0  # mm
    r = robot_width/2.0

    # Compute first orientation.
    theta_1 = np.arctan2(y, x)
    s1 = r*theta_1
    l1 = -s1
    r1 = s1

    # Compute drive length.
    d = la.norm([x, y])
    l2 = d
    r2 = d

    # Compute second orientation.
    theta_2 = theta - theta_1
    s2 = r*theta_2
    l3 = -s2
    r3 = s2

    # return [[l1, r1], [l2, r2], [l3, r3]]
    return [[l1 + l2 + l3, r1 + r2 + r3]]
