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

# import time
import serial
import numpy as np
from leg_utils import extract_legs_from_scan
from control.command_robot import goto_rel_pose


def setup_robot(port_name):
    # Open the serial port.
    ser = serial.Serial(port_name)
    print(ser.name)
    # Play sound to indicate script start.
    ser.write('\rPlaySound soundid 1\r')
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
    ser.close()


def teardown_robot(port_name):
    # Open the serial port.
    ser = serial.Serial(port_name)
    ser.write('TestMode Off\r')
    ser.close()


def get_scan(port_name):
    # Open the serial port.
    ser = serial.Serial(port_name)

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

    ranges = np.array(ranges)
    angles = np.radians(angles)

    return [ranges, angles]


def get_odom(port_name):
    # Open the serial port.
    ser = serial.Serial(port_name)

    # Flush buffer.
    line_tok = ['start']
    while line_tok[0] != 'Parameter':
        line = ser.readline()
        line_tok = line.split(',')

    # Read motor data.
    w_l = -1
    w_r = -1
    while line_tok[0] != 'RightWheel_PositionInMM':
        line = ser.readline()
        line_tok = line.split(',')
        if line_tok[0] != 'LeftWheel_PositionInMM':
            w_l = float(line_tok[1])
        elif line_tok[0] != 'RightWheel_PositionInMM':
            w_r = float(line_tok[1])

    return [w_l, w_r]


def send_motor_cmd(port_name, left_wheel_in_mm, right_wheel_in_mm, vel):
    # Open the serial port.
    ser = serial.Serial(port_name)
    # Play sound to indicate script start.
    ser.write('\rSendMotor ' + str(left_wheel_in_mm) + str(right_wheel_in_mm) + str(vel) + '\r')
    ser.close()


def command_to_rel_pose(port_name, x, y, theta):
    # Get commands.
    cmds = goto_rel_pose(x, x, theta)

    # Get current odom.
    [w_l0, w_r0] = get_odom(port_name)

    # Send first command.
    send_motor_cmd(port_name, cmds[0][0], cmds[0][1], 100)

    w_l = w_l0
    w_r = w_r0
    while cmds[0][0] - (w_l - w_l0) > 0 and \
            cmds[0][1] - (w_r - w_r0) > 0:
        [w_l, w_r] = get_odom(port_name)

    # Get current odom.
    [w_l0, w_r0] = get_odom(port_name)

    # Send second command.
    send_motor_cmd(port_name, cmds[1][0], cmds[1][1], 100)

    w_l = w_l0
    w_r = w_r0
    while cmds[1][0] - (w_l - w_l0) > 0 and \
            cmds[1][1] - (w_r - w_r0) > 0:
        [w_l, w_r] = get_odom(port_name)

    # Get current odom.
    [w_l0, w_r0] = get_odom(port_name)

    # Send third command.
    send_motor_cmd(port_name, cmds[2][0], cmds[2][1], 100)

    w_l = w_l0
    w_r = w_r0
    while cmds[2][0] - (w_l - w_l0) > 0 and \
            cmds[2][1] - (w_r - w_r0) > 0:
        [w_l, w_r] = get_odom(port_name)


def main():
    # Setup robot.
    port_name = '/dev/ttyACM0'
    setup_robot(port_name)

    # Get scan.
    [ranges, angles] = get_scan(port_name)

    # Find the legs.
    [cens, r_seg, a_seg] = extract_legs_from_scan(ranges, angles)

    # Get commands.
    command_to_rel_pose(port_name, 500, -200, -np.pi)

    # We're done.
    teardown_robot(port_name)
    print('Done.')

if __name__ == '__main__':
    main()
