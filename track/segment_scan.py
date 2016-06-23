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
from numpy import linalg as la


def shortest_angular_distance(ang1, ang2):
    """Returns smallest angle between two angles. Assumes radians, result is always positive."""
    da = np.absolute(ang1 - ang2)
    if da > np.pi:
        da = np.absolute(da - 2*np.pi)

    return da


def filter_seg_gt(range_segs, bear_segs, max_n):
    """Removes segments that have more than max_n elements."""
    r_segs = []
    b_segs = []

    for r_set, b_set in zip(range_segs, bear_segs):
        if r_set.size <= max_n:
            r_segs.append(r_set)
            b_segs.append(b_set)

    return [r_segs, b_segs]


def filter_seg_wider(range_segs, bear_segs, width_mm):
    """Removes segments whose width is greater than some amount."""
    r_segs = []
    b_segs = []

    for r_set, b_set in zip(range_segs, bear_segs):
        # Convert to cartesian.
        x_set = [r_set[0]*np.cos(b_set[0]), r_set[-1]*np.cos(b_set[-1])]
        y_set = [r_set[0]*np.sin(b_set[0]), r_set[-1]*np.sin(b_set[-1])]

        # Get distance from first to last.
        d = la.norm([x_set[0] - x_set[-1], y_set[0] - y_set[-1]])

        if d <= width_mm:
            r_segs.append(r_set)
            b_segs.append(b_set)

    return [r_segs, b_segs]


def get_centroids(range_segs, bear_segs):
    """Takes list of range and bearing segments and returns a list of their x,y centroids."""
    centroids = []

    for r_set, b_set in zip(range_segs, bear_segs):
        # Convert to cartesian.
        x_set = r_set*np.cos(b_set)
        y_set = r_set*np.sin(b_set)
        # Get centroid.
        x = np.mean(x_set)
        y = np.mean(y_set)

        centroids.append([x, y])

    return centroids


class Segmenter:
    def __init__(self):
        self._ranges = np.array([])
        self._bearings = np.array([])
        self._r_segs = np.array([])
        self._b_segs = np.array([])
        self._dirty = True

    def add_scan(self, ranges, bearings):
        self._ranges = np.array(ranges)
        self._bearings = np.array(bearings)

        # Filter out too close and too far.
        [self._ranges, self._bearings] = self._filter_scan(self._ranges, self._bearings)
        self._dirty = True

    def _filter_scan(self, ranges, bearings):
        # Filter out too close and too far.
        # r_min = 0.15  # meters
        # r_max = 4.0   # meters
        r_min = 150  # mm
        r_max = 4000   # mm
        bearings = bearings[(ranges >= r_min) & (ranges <= r_max)]
        ranges = ranges[(ranges >= r_min) & (ranges <= r_max)]
        return [ranges, bearings]

    def get_segments(self):
        if self._dirty:
            # Segment the scan.
            [self._r_segs, self._b_segs] = self._compute_segments(self._ranges, self._bearings)

            # Filter scan.
            [self._r_segs, self._b_segs] = self._filter_segments(self._r_segs, self._b_segs)

            self._dirty = False
        return [self._r_segs, self._b_segs]

    def _filter_segments(self, range_segs, bear_segs):
        # Remove small clusters.
        rf_segs = []
        bf_segs = []
        for r_seg, b_seg in zip(range_segs, bear_segs):
            if r_seg.size > 2:
                rf_segs.append(r_seg)
                bf_segs.append(b_seg)

        return [rf_segs, bf_segs]

    def _compute_segments(self, ranges, bearings):
        # If the range between neighbors is too great, break into segments.
        # Get the differences between ranges.
        dr = np.diff(ranges)
        dth = np.diff(bearings)

        # Find the indices where the difference is high.
        # Adding one to make it the index first element of the new set.
        # r_diff = 0.2  # meters
        r_diff = 200    # mm
        th_diff = 0.17  # radians
        dr_ndxs = np.where(np.absolute(dr) > r_diff)[0] + 1
        dth_ndxs = np.where(np.absolute(dth) > th_diff)[0] + 1
        d_ndxs = np.concatenate([dr_ndxs, dth_ndxs])
        d_ndxs.sort()

        # Get the sets.
        r_segs = np.split(ranges, d_ndxs)
        b_segs = np.split(bearings, d_ndxs)

        # Check for wrap arounds.
        if (np.absolute(r_segs[0][0] - r_segs[-1][-1]) <= r_diff) and \
                (shortest_angular_distance(b_segs[0][0], b_segs[-1][-1]) <= th_diff):
            # Add the first set to the last set and get rid of the first.
            r_segs[-1] = np.concatenate([r_segs[-1], r_segs[0]])
            b_segs[-1] = np.concatenate([b_segs[-1], b_segs[0]])
            r_segs = np.delete(r_segs, 0)
            b_segs = np.delete(b_segs, 0)

        return [r_segs, b_segs]        
