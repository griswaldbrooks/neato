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

## @file display_lds_scan.py Script for displaying lds scan from file.

import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import argparse
from leg_utils import extract_legs_from_scan


def main():
    # Get command line args.
    parser = argparse.ArgumentParser()
    parser.add_argument('scan')
    args = parser.parse_args()

    # Grab the data out of the file.
    scan_log = np.genfromtxt(args.scan, delimiter=',')

    ax = plt.subplot(111, projection='polar')
    ax.scatter(np.radians(scan_log[:, 0]), scan_log[:, 1], color='0.75')
    ax.set_rmax(6000)
    ax.grid(True)

    [cens, r_seg, a_seg] = extract_legs_from_scan(scan_log[:, 1], np.radians(scan_log[:, 0]))

    colors = cm.prism(np.random.rand(len(r_seg), 1))
    for r, a, ctr, c in zip(r_seg, a_seg, cens, colors):
        ax.scatter(a, r, color=c)
        ax.scatter(np.arctan2(ctr[1], ctr[0]), la.norm(ctr), color=c, marker=">", edgecolor='black')
        print r, a
    ###

    ax.set_title("A line plot on a polar axis", va='bottom')
    plt.show()

if __name__ == '__main__':
    main()
