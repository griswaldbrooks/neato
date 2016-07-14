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

## @file leg_utils.py Module for finding legs in laser scans.

from segment_scan import Segmenter
from segment_scan import get_centroids
from segment_scan import filter_seg_gt
from segment_scan import filter_seg_wider


def extract_legs_from_scan(ranges, angles):
    # Segment scan.
    sgr = Segmenter()
    sgr.add_scan(ranges, angles)
    [r_seg, a_seg] = sgr.get_segments()

    # Do basic filtering
    max_num = 40
    max_width = 400  # mm
    [r_seg, a_seg] = filter_seg_gt(r_seg, a_seg, max_num)
    [r_seg, a_seg] = filter_seg_wider(r_seg, a_seg, max_width)
    cens = get_centroids(r_seg, a_seg)

    return [cens, r_seg, a_seg]
