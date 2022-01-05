#!/usr/bin/env python


import sys
import yaml
import rospy  # type: ignore
import argparse
import numpy as np

from math import cos, sin
from datetime import datetime
from os.path import exists as file_exists
from niryo_one_python_api.niryo_one_api import *  # type: ignore

sys.path.insert(1, "/home/vaco/catkin_ws/src/niryo_one_python_api")  # type: ignore

rospy.init_node("niryo_one_example_python_api")

# joint 1 transformation
def T_01(t1):
    R_t = np.array([[cos(t1), -sin(t1), 0], [sin(t1), cos(t1), 0], [0, 0, 1]])

    R_0 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    R = np.matmul(R_t, R_0)

    T = np.array([0, 0, 0.103])

    return np.array(
        [
            [R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]],
            [0, 0, 0, 1],
        ]
    )


# joint 2 transformation
def T_12(t2):
    R_t = np.array([[cos(-t2), 0, sin(-t2)], [0, 1, 0], [-sin(-t2), 0, cos(-t2)]], dtype=float)

    R_0 = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)

    R = np.matmul(R_t, R_0)

    T = np.array([0, 0, 0.08], dtype=float)

    return np.array(
        [
            [R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


# joint 3 transformation
def T_23(t3):
    R_t = np.array([[cos(t3), -sin(t3), 0], [sin(t3), cos(t3), 0], [0, 0, 1]], dtype=float)

    R_0 = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)

    R = np.matmul(R_t, R_0)

    T = np.array([0.21, 0, 0], dtype=float)

    return np.array(
        [
            [R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


# joint 4 transformation
def T_34(t4):
    R_t = np.array([[1, 0, 0], [0, cos(t4), -sin(t4)], [0, sin(t4), cos(t4)]], dtype=float)

    R_0 = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)

    R = np.matmul(R_t, R_0)

    T = np.array([0.0415, 0.03, 0], dtype=float)

    return np.array(
        [
            [R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


# joint 2 transformation
def T_45(t5):
    R_t = np.array([[1, 0, 0], [0, cos(-t5), -sin(-t5)], [0, sin(-t5), cos(-t5)]], dtype=float)

    R_0 = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], dtype=float)

    R = np.matmul(R_t, R_0)

    T = np.array([0, 0, 0.18], dtype=float)

    return np.array(
        [
            [R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


# joint 6 transformation
def T_56(t6):
    R_t = np.array([[1, 0, 0], [0, cos(t6), -sin(t6)], [0, sin(t6), cos(t6)]], dtype=float)

    R_0 = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)

    R = np.matmul(R_t, R_0)

    T = np.array([0.0237, -0.0055, 0], dtype=float)

    return np.array(
        [
            [R[0][0], R[0][1], R[0][2], T[0]],
            [R[1][0], R[1][1], R[1][2], T[1]],
            [R[2][0], R[2][1], R[2][2], T[2]],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


def theoretic_position(theta):
    T = np.matmul(
        T_01(theta[0]),
        np.matmul(
            T_12(theta[1]), np.matmul(T_23(theta[2]), np.matmul(T_34(theta[3]), np.matmul(T_45(theta[4]), T_56(theta[5]))))
        ),
    )

    print("Theoretic position:")
    print("  x: " + str(T[0][3]))
    print("  y: " + str(T[1][3]))
    print("  z: " + str(T[2][3]))
    print("")

try:
    pi = 3.14159
    n = NiryoOne()  # type: ignore

    n.calibrate_auto()

    joints = [0, 0, pi/2, 0, 0, 0]

    n.move_joints(joints)

    theoretic_position(joints)

    print(n.get_arm_pose())
except Exception as e:
    print(e)
