#!/usr/bin/env python
#
# Software Licence Agreement (MIT)
#
# Copyright (c) 2016 Griswald Brooks
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
#

##
# @author Griswald Brooks

# @file motion_orient.py Script for tracking movement by orienting to them.

import time
import serial
from time import sleep
import numpy as np
from numpy import linalg as la
from control.command_robot import get_wheel_cmds_to_rel_pose_triple

from picamera import PiCamera


def setup_robot(port_name):
    # Open the serial port.
    ser = serial.Serial(port_name)
    print(ser.name)
    # Play sound to indicate script start.
    ser.write('\rPlaySound soundid 1\r')
    print(ser.readline())
    # Setup test mode.
    ser.write('TestMode On\r')
    print(ser.readline())
    ser.write('SetLDSRotation On\r')
    print(ser.readline())
    ser.write('SetMotor LWheelEnable\r')
    print(ser.readline())
    ser.write('SetMotor RWheelEnable\r')
    print(ser.readline())
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

    # Get odom.
    ser.write('getmotor\r')

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
        if line_tok[0] == 'LeftWheel_PositionInMM':
            w_l = float(line_tok[1])
        elif line_tok[0] == 'RightWheel_PositionInMM':
            w_r = float(line_tok[1])

    return [w_l, w_r]


def get_pose(port_name):
    # Open the serial port.
    ser = serial.Serial(port_name)

    # Get odom.
    ser.write('getrobotpos smooth\r')

    # Flush buffer.
    line_tok = ['start']
    while line_tok[0] != 'Robot Smooth pose':
        line = ser.readline()
        line_tok = line.split(':')

    pose_str = line_tok[1].split(',')
    x = float(pose_str[0].split('=')[1])
    y = float(pose_str[1].split('=')[1])
    theta = float(pose_str[2].split('=')[1])

    return [x, y, theta]


def send_motor_cmd(port_name, left_wheel_in_mm, right_wheel_in_mm, vel):
    # Open the serial port.
    ser = serial.Serial(port_name)
    # Send command.
    ser.write('\rsetmotor lwheeldist ' + str(left_wheel_in_mm) +
              ' rwheeldist ' + str(right_wheel_in_mm) +
              ' speed ' + str(vel) + '\r')
    ser.close()


def wait_for_drive_completion(port_name, cmd_w_l, cmd_w_r):
    """Check wheel odometry and return when drive command is complete."""
    [init_w_l, init_w_r] = get_odom(port_name)
    error_l = float("inf")
    error_r = float("inf")
    eps = 2  # Error tolerance in mm.
    pose = np.array([0.0, 0.0, 0.0])
    w_lp = init_w_l
    w_rp = init_w_r
    while error_l > eps and error_r > eps:
        [w_l, w_r] = get_odom(port_name)
        error_l = np.absolute(cmd_w_l - (w_l - init_w_l))
        error_r = np.absolute(cmd_w_r - (w_r - init_w_r))
        pose += get_change_in_pose(w_l - w_lp, w_r - w_rp)
        w_lp = w_l
        w_rp = w_r

    dw_l = w_l - init_w_l
    dw_r = w_r - init_w_r
    pose = get_change_in_pose(dw_l, dw_r)
    # TODO: Add a timeout.
    return pose


def command_to_rel_pose(port_name, x, y, theta):
    # Get commands.
    cmds = get_wheel_cmds_to_rel_pose_triple(x, y, theta)

    for cmd in cmds:
        send_motor_cmd(port_name, cmd[0], cmd[1], 200)
        wait_for_drive_completion(port_name, cmd[0], cmd[1])


def shortest_angular_distance(angle1, angle2):
    """
    Finds the signed smallest angles between two angles.
    Returns angle2 wrt angle1 as in:
        angle1 = 45 angle2 = 30 returns -15

    """
    da = angle2 - angle1
    return (da + 180.0) % 360.0 - 180.0


def command_to_orientation(port_name, theta, speed):
    goal_theta = theta
    theta_err = goal_theta
    [x, y, robot_theta] = get_pose(port_name)
    dt = 0.1
    while np.absolute(theta_err) > 1.0:
        [x, y, robot_theta] = get_pose(port_name)
        print("robot_theta = " + str(robot_theta))
        # Pose error.
        theta_err = shortest_angular_distance(robot_theta, goal_theta)
        print("theta_err = " + str(theta_err))
        s_l = -2.0*theta_err
        s_r = 2.0*theta_err
        send_motor_cmd(port_name, s_l, s_r, speed)
        sleep(dt)

    send_motor_cmd(port_name, 1, 1, 1)
    send_motor_cmd(port_name, 0, 0, 0)
    sleep(0.5)


def command_to_rel_position(port_name, x, y, speed):
    robot_width = 245.0  # mm
    robot_radius = robot_width/2.0
    robot_pose = np.array([0.0, 0.0, 0.0])
    goal_pose = np.array([x, y, 0.0])
    dt = 0.1
    [init_w_l, init_w_r] = get_odom(port_name)
    pose_err = goal_pose - robot_pose
    while la.norm(pose_err[0:2]) > 50.0:
        # Compute relative pose.
        [w_l, w_r] = get_odom(port_name)
        dw_l = w_l - init_w_l
        dw_r = w_r - init_w_r
        robot_pose = get_change_in_pose(dw_l, dw_r)
        # Pose error.
        pose_err = goal_pose - robot_pose

        # Compute orientation.
        theta = np.arctan2(pose_err[1], pose_err[0])
        s_theta = robot_radius*theta

        # Compute drive length.
        d = la.norm([x, y])

        s_l = -s_theta + d
        s_r = s_theta + d
        send_motor_cmd(port_name, s_l, s_r, speed)
        sleep(dt)


def get_change_in_pose(s_l, s_r):
    """Get the change in [x, y, theta] given the wheel distances."""
    # Check for driving straight.
    if s_l == s_r:
        dx = s_l
        dy = 0
        dtheta = 0
    else:
        robot_width = 245.0  # mm
        robot_radius = robot_width/2.0

        # Compute turning radius and turning angle.
        R = robot_radius*(s_l + s_r)/(s_l - s_r)
        gamma = (s_l - s_r)/robot_width

        # Compute change in pose.
        dx = R*np.sin(gamma)
        dy = R*(np.cos(gamma) - 1)
        dtheta = -gamma

    return np.array([dx, dy, dtheta])


def scans_are_different(ranges1, ranges2, thresh=1000):
    """
    Function to compare two LDS scans and see if they are different enough.

    Args:
        ranges1 (np.array): An array of ranges from the LDS.
        ranges2 (np.array): An array of ranges from the LDS.
        thresh (float): The error threshold. If the error is above this number,
                        the scans are different.

    Returns:
        bool: True of the scans are different.
    """
    r_min = 150  # mm
    r_max = 5000   # mm
    bad_ndxs = [ranges1 < r_min,
                ranges1 > r_max,
                ranges2 < r_min,
                ranges2 > r_max]
    for bdx in bad_ndxs:
        ranges1[bdx] = 0
        ranges2[bdx] = 0
    err = la.norm(ranges1 - ranges2)
    if err > thresh:
        return True

    return False


def find_big_diff_pt(ranges1, ranges2):
    dr = np.absolute(ranges1 - ranges2)
    ndx = np.argmax(dr)
    # Assuming index maps to angle in degrees.
    pt = np.array([np.cos(ndx), np.sin(ndx)])
    pt *= ranges1[ndx]

    return [pt, ndx]


def main():
    # Setup robot.
    port_name = '/dev/ttyACM0'
    setup_robot(port_name)

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    camera.vflip = True
    camera.hflip = True

    # Scan to compare other to for motions.
    time.sleep(10)
    print("Watching.")
    [base_ranges, angles] = get_scan(port_name)
    while not la.norm(base_ranges):
        [base_ranges, angles] = get_scan(port_name)

    while True:
        # Get scan.
        [ranges, angles] = get_scan(port_name)

        if scans_are_different(base_ranges, ranges):
            ser = serial.Serial(port_name)
            ser.write('\rPlaySound soundid 1\r')
            ser.close()
            [point, rel_angle] = find_big_diff_pt(base_ranges, ranges)
            [x, y, robot_theta] = get_pose(port_name)
            theta = rel_angle + robot_theta
            print("rel_angle = " + str(rel_angle))
            print("robot_theta = " + str(robot_theta))
            print("theta = " + str(theta))
            command_to_orientation(port_name, theta, 200)
            ser = serial.Serial(port_name)
            ser.write('\rPlaySound soundid 3\r')
            ser.close()
            [base_ranges, angles] = get_scan(port_name)
            foldername = "images"
            picturename = str(time.clock())
            camera.capture(foldername + '/' + picturename + '.jpg')

    # We're done.
    teardown_robot(port_name)
    print('Done.')

if __name__ == '__main__':
    main()
