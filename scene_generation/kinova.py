#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
import sys
import time
import threading
import multiprocessing

import numpy as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2

import utilities

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

stdout = sys.stdout


def check_for_end_or_abort(e, verbose=False):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification):
        if verbose:
            if notification.action_event == 11:
                print("EVENT : ACTION_FEEDBACK")
            else:
                print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))

        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check


class Kinova:
    def __init__(self, router, verbose=False):
        # Create connection to the device and get the router

        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router)

        self.verbose = verbose

    def move_to_starting_position(self):
        self.move_to_joint_angles([32, 265, 53, 266, 351, 288, 325])

    def move_to_home_position(self):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)

        # Move arm to the home position
        if self.verbose:
            print("Moving the arm to the home position")

        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle is None:
            if self.verbose:
                print("Can't reach the home position. Exiting")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(e, verbose=self.verbose),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            if self.verbose:
                print("Safe position reached")
        else:
            if self.verbose:
                print("Timeout on action notification wait")

        return finished

    def move_to_joint_angles(self, target_angles):
        if self.verbose:
            print("Starting angular action movement ...")

        action = Base_pb2.Action()
        action.name = "Angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = target_angles[joint_id]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(e, verbose=self.verbose),
            Base_pb2.NotificationOptions()
        )

        if self.verbose:
            print("Executing action")
        self.base.ExecuteAction(action)

        if self.verbose:
            print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            if self.verbose:
                print("Angular movement completed")
        else:
            if self.verbose:
                print("Timeout on action notification wait")

        return finished

    def move_to_cartesian_pose(self, target_pose):
        if self.verbose:
            print("Starting Cartesian action movement ...")

        action = Base_pb2.Action()
        action.name = "Cartesian action movement"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = target_pose[0]  # (meters)
        cartesian_pose.y = target_pose[1]  # (meters)
        cartesian_pose.z = target_pose[2]  # (meters)
        cartesian_pose.theta_x = target_pose[3]  # (degrees)
        cartesian_pose.theta_y = target_pose[4]  # (degrees)
        cartesian_pose.theta_z = target_pose[5]  # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(e, verbose=self.verbose),
            Base_pb2.NotificationOptions()
        )

        if self.verbose:
            print("Executing action")
        self.base.ExecuteAction(action)

        if self.verbose:
            print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            if self.verbose:
                print("Cartesian movement completed")
        else:
            if self.verbose:
                print("Timeout on action notification wait")

        return finished

    def get_cartesian_pose(self):
        base = self.base_cyclic.RefreshFeedback().base

        return [base.tool_pose_x, base.tool_pose_y, base.tool_pose_z,
                base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]


def generate_poses(arc_radius_inches=24, min_angle=10, max_angle=80, num_steps=15, ):
    def inch2meter(inch):
        return 0.0254 * inch

    z_offset = 0.012
    turntable_position = [inch2meter(24), 0, 0]  # meter
    arc_radius = inch2meter(arc_radius_inches)  # meter
    end_effector_offset = inch2meter(12)  # meter
    end_effector_offset_angle = np.arctan(end_effector_offset / turntable_position[0])  # rad
    azure_offset = 0.096  # meter

    all_theta_y_deg = list(range(min_angle, max_angle + 1, int((max_angle - min_angle) / (num_steps - 2))))
    all_theta_y_rad = np.deg2rad(all_theta_y_deg)
    end_poses = np.zeros([len(all_theta_y_rad), 6])

    for i, theta in enumerate(all_theta_y_rad):
        azure_offset_x = np.sin(theta) * azure_offset  # meter
        azure_offset_z = np.cos(theta) * azure_offset  # meter
        arc_projection = arc_radius * np.cos(theta)  # meter

        end_poses[i][0] = turntable_position[0] - arc_projection * np.cos(
            end_effector_offset_angle) - azure_offset_x  # meter
        end_poses[i][1] = arc_projection * np.sin(end_effector_offset_angle)  # meter
        end_poses[i][2] = arc_radius * np.sin(theta) - z_offset - azure_offset_z  # meter
        end_poses[i][3] = np.rad2deg(np.pi / 2 + theta)  # deg
        end_poses[i][4] = 0  # deg
        end_poses[i][5] = np.rad2deg(np.pi / 2 - end_effector_offset_angle)  # deg

    return end_poses


def robot_to_starting_position(connection_args, dummy_arg=None, verbose=False):
    if not verbose:
        sys.stdout = None

    with utilities.DeviceConnection.createTcpConnection(connection_args) as router:
        Kinova(router, verbose=verbose).move_to_starting_position()

    if not verbose:
        sys.stdout = stdout


def robot_to_home_position(connection_args, dummy_arg=None, verbose=False):
    if not verbose:
        sys.stdout = None

    with utilities.DeviceConnection.createTcpConnection(connection_args) as router:
        Kinova(router, verbose=verbose).move_to_home_position()

    if not verbose:
        sys.stdout = stdout


def robot_to_joint_angles(connection_args, target_angles, verbose=False):
    if not verbose:
        sys.stdout = None

    with utilities.DeviceConnection.createTcpConnection(connection_args) as router:
        Kinova(router, verbose=verbose).move_to_joint_angles(target_angles)

    if not verbose:
        sys.stdout = stdout


def robot_to_cartesian_pose(connection_args, target_pose, verbose=False):
    if not verbose:
        sys.stdout = None

    with utilities.DeviceConnection.createTcpConnection(connection_args) as router:
        Kinova(router, verbose=verbose).move_to_cartesian_pose(target_pose)

    if not verbose:
        sys.stdout = stdout


def get_robot_y(connection_args, verbose=False):
    if not verbose:
        sys.stdout = None

    with utilities.DeviceConnection.createTcpConnection(connection_args) as router:
        robot_y = Kinova(router, verbose=verbose).get_cartesian_pose()[1]

    if not verbose:
        sys.stdout = stdout

    return robot_y


def multiprocessing_test(args):
    tcp_connection = utilities.DeviceConnection(ipAddress=args[0], port=args[1], use_tcp=True,
                                                credentials=(args[2], args[3]))
    kinova_router = tcp_connection.open()
    kinova = Kinova(kinova_router, verbose=True)
    kinova.move_to_home_position()
    kinova.move_to_starting_position()

    poses = generate_poses(arc_radius_inches=28, max_angle=80, num_steps=15, )
    print(len(poses))

    for idx, pose in enumerate(poses):
        print()
        print("Angle #", idx, ": ", pose[3] - 90)
        kinova.move_to_cartesian_pose(pose)
        time.sleep(1)

    kinova.move_to_home_position()

    tcp_connection.close()


if __name__ == "__main__":
    args1 = ["192.168.1.10", 10000, "admin", "admin"]
    args2 = ["192.168.1.12", 10000, "admin", "admin"]

    p1 = multiprocessing.Process(target=multiprocessing_test, args=(args1,))
    p2 = multiprocessing.Process(target=multiprocessing_test, args=(args2,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()
