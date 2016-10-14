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
from track.leg_utils import extract_legs_from_scan
# from track.segment_scan import Segmenter
# from track.segment_scan import get_centroids
# from track.segment_scan import filter_seg_gt
# from track.segment_scan import filter_seg_wider
from control.command_robot import get_wheel_cmds_to_rel_pose_triple


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


def command_to_orientation(port_name, x, y, speed):
    robot_width = 245.0  # mm
    robot_radius = robot_width/2.0
    theta_err = np.arctan2(y, x)
    robot_pose = np.array([0.0, 0.0, 0.0])
    goal_pose = np.array([x, y, theta_err])
    dt = 0.1
    [init_w_l, init_w_r] = get_odom(port_name)
    pose_err = goal_pose - robot_pose
    while np.absolute(theta_err) > 0.2:
        # Compute relative pose.
        [w_l, w_r] = get_odom(port_name)
        dw_l = w_l - init_w_l
        dw_r = w_r - init_w_r
        robot_pose = get_change_in_pose(dw_l, dw_r)
        # Pose error.
        pose_err = goal_pose - robot_pose
        theta_err = pose_err[2]
        # Compute orientation.
        theta = np.arctan2(pose_err[1], pose_err[0])
        s_theta = robot_radius*theta

        s_l = -s_theta
        s_r = s_theta
        send_motor_cmd(port_name, s_l, s_r, speed)
        sleep(dt)


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

    return pt


def main():
    # Setup robot.
    port_name = '/dev/ttyACM0'
    setup_robot(port_name)

    # Person starting pose.
    person_pose = np.array([500, 0, 0])
    tracking = False
    misses = 0
    # Scan to compare other to for motions.
    track = False
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
            point = find_big_diff_pt(base_ranges, ranges)
            command_to_orientation(port_name, point[0], point[1], 100)
            base_ranges = ranges

        if track:
            # Find the legs.
            if ranges.any():
                [cens, r_seg, a_seg] = extract_legs_from_scan(ranges, angles)
                print('cens = ' + str(cens))

                # Find the centroid closest to the person.
                cens_s = sorted(cens,
                                key=lambda cand: la.norm(cand - person_pose[0:2]))
                if cens_s:
                    # If the centroid is within some range of the previous,
                    # then track it.
                    d_range = la.norm(cens_s[0] - person_pose[0:2])
                    if d_range < 200:
                        # If this is the first time tracking, play the sound.
                        if not tracking:
                            ser = serial.Serial(port_name)
                            ser.write('\rPlaySound soundid 1\r')
                            ser.close()
                            tracking = True
                            misses = 0

                        # Set pose.
                        person_pose = np.array([cens_s[0][0], cens_s[0][1], 0])
                        print('person_pose = ' + str(person_pose))

                        # Get commands.
                        # command_to_rel_pose(port_name, 500, -200, -np.pi/2)
                        # command_to_rel_position(port_name, 500, -200)
                        person_range = la.norm(person_pose[0:2])
                        print('person_range = ' + str(person_range))
                        if (person_range > 500.0) and (person_range < 2500.0):
                            # Range and speed limits.
                            rng = [500, 2500]
                            spd = [75, 350]
                            speed = np.interp(person_range, rng, spd)
                            # print('speed = ' + str(speed))
                            command_to_rel_position(port_name,
                                                    person_pose[0]*0.1,
                                                    person_pose[1]*0.1,
                                                    speed)

                    # If it is outside the range, lose tracking,
                    else:
                        # Play the lost tracking sound.
                        if tracking and (misses > 5):
                            ser = serial.Serial(port_name)
                            ser.write('\rPlaySound soundid 3\r')
                            ser.close()
                            tracking = False
                            # Reset pose.
                            person_pose = np.array([500, 0, 0])
                            # Stop the robot.
                            command_to_rel_position(port_name, 0, 0, 0)
                        misses += 1
                        print('person_pose = ' + str(person_pose))

    # We're done.
    teardown_robot(port_name)
    print('Done.')

if __name__ == '__main__':
    main()
