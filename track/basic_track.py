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

## @file basic_track.py Script for doing basic cluster tracking.

import time
import serial
import numpy as np


def main():
    # Open the serial port.
    ser = serial.Serial('/dev/ttyACM0')
    print(ser.name)
    # Play sound to indicate script start.
    ser.write('\rPlaySound 1\r')
    ser.readline()
    # Setup test mode.
    ser.write('TestMode On\r')
    ser.readline()
    ser.write('SetLDSRotation On\r')
    ser.readline()
    ser.write('SetMotor LWheelEnable\r')
    ser.readline()
    ser.write('SetMotor RWheelEnable\r')
    ser.readline()

    # Get scan.
    ser.write('GetLDSScan\r')
    ser.readline()

    # Flush buffer.
    line_tok = ['start']
    while line_tok[0] != 'AngleInDegrees':
        line = ser.readline()
        line_tok = line.split(',')

    # Read scan data.
    angles = []
    ranges = []
    while line_tok[0] != 'ROTATION_SPEED':
        line = ser.readline()
        line_tok = line.split(',')
        if line_tok[0] != 'ROTATION_SPEED':
            angles.append(float(line_tok[0]))
            ranges.append(float(line_tok[1]))

    ranges = np.array(ranges)/1000.0
    angles = np.array(angles)*np.pi/180.0

    print ranges
    print angles

    # We're done.
    ser.write(b'TestMode Off\r')
    ser.close()
    print('Done.')

if __name__ == '__main__':
    main()
