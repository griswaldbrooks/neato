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

## @file convert_polar_to_cart.py Script for converting polar laser scan to cartesian coordinates.

import numpy as np
import argparse


def main():
    # Get command line args.
    parser = argparse.ArgumentParser()
    parser.add_argument('scan')
    args = parser.parse_args()

    # Grab the data out of the file.
    scan_log = np.genfromtxt(args.scan, delimiter=',')

    angles = np.radians(scan_log[:, 0])
    ranges = scan_log[:, 1]/1000.0
    x = ranges*np.cos(angles)
    y = ranges*np.sin(angles)

    np.savetxt('cart' + args.scan, np.transpose([x, y]), delimiter=',')

if __name__ == '__main__':
    main()
