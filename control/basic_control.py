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

## @file basic_control.py Script for doing basic teleop with the Botvac.

import serial


def main():
    # Open the serial port.
    ser = serial.Serial('/dev/ttyACM0')
    print(ser.name)
    # Play sound to indicate script start.
    ser.write(b'\rPlaySound soundid 1\r')
    # Setup test mode.
    ser.write(b'TestMode On\r')
    ser.write(b'SetMotor LWheelEnable\r')
    ser.write(b'SetMotor RWheelEnable\r')
    # Take basic commands.
    cmd = ''
    while cmd is not 'q':
        cmd = raw_input('>>>')
        if cmd is 'w':
            ser.write(b'SetMotor lwheeldist 100 rwheeldist 100 speed 200 0\r')
        elif cmd is 'ww':
            ser.write(b'SetMotor lwheeldist 1000 rhweeldist 1000 speed 100 0\r')
        elif cmd is 's':
            ser.write(b'SetMotor lwheeldist -100 rwheeldist -100 speed 200 0\r')
        elif cmd is 'ss':
            ser.write(b'SetMotor lwheeldist -1000 rwheeldist -1000 speed 100 0\r')
        elif cmd is 'a':
            ser.write(b'SetMotor lwheeldist -100 rwheeldist 100 speed 200 0\r')
        elif cmd is 'aa':
            ser.write(b'SetMotor lwheeldist -1000 rwheeldist 1000 speed 100 0\r')
        elif cmd is 'd':
            ser.write(b'SetMotor lwheeldist 100 rwheeldist -100 speed 200 0\r')
        elif cmd is 'dd':
            ser.write(b'SetMotor lwheeldist 1000 rwheeldist -1000 speed 100 0\r')

    # We're done.
    ser.write(b'TestMode Off\r')
    ser.close()

if __name__ == '__main__':
    main()
