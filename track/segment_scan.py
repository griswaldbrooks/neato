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

## @file segment_scan.py Module for segmenting laser scans.

import numpy as np


class Segmenter:
    def __init__(self):
        self._ranges = np.array([])
        self._bearings = np.array([])

    def add_scan(self, ranges, bearings):
        self._ranges = np.array(ranges)
        self._bearings = np.array(bearings)

        # Filter out too close and too far.
        # r_min = 0.15  # meters
        # r_max = 4.0   # meters
        r_min = 150  # mm
        r_max = 4000   # mm
        self._bearings = self._bearings[(self._ranges >= r_min) & (self._ranges <= r_max)]
        self._ranges = self._ranges[(self._ranges >= r_min) & (self._ranges <= r_max)]

    def get_segments(self):
        # If the range between neighbors is too great, break into segments.
        # Get the differences between ranges.
        dr = np.diff(self._ranges)
        # Find the indices where the difference is high.
        # Adding one to make it the index first element of the new set.
        # r_diff = 0.2  # meters
        r_diff = 200  # mm
        d_ndxs = np.where(np.absolute(dr) > r_diff)[0] + 1
        # Get the sets.
        r_segs = np.split(self._ranges, d_ndxs)
        b_segs = np.split(self._bearings, d_ndxs)

        return [r_segs, b_segs]
