#!/usr/bin/env python


import sys
import yaml
from yaml.loader import Loader
import rospy  # type: ignore

from datetime import datetime
from niryo_one_python_api.niryo_one_api import *  # type: ignore

sys.path.insert(1, "/home/vaco/catkin_ws/src/niryo_one_python_api")  # type: ignore

rospy.init_node("niryo_one_example_python_api")

pi = 3.14159
n = NiryoOne()  # type: ignore

actions = []
start_time = datetime.now()

# Auxiliary functions
def load_params(config_file):
    params = {}

    if not config_file:
        raise SystemExit("A file with the points must be specified")

    with open(config_file + ".yaml") as parameters:
        params = yaml.safe_load(parameters)

    return params


def stringify(array):
    array = [array] if isinstance(array, int) else array

    return ",".join([str(x) for x in array])


def log_action(timestamp, action, value):
    global actions
    global start_time

    # EXAMPLE ->  0:00:03.716501,MOVE_JOINTS:0.03,0.0123,0.456,0.987,0.654,0.321
    actions.append(str(timestamp - start_time) + "," + action + ":" + value)


# Functions to change robot parameters
def calibrateRobot():
    n.calibrate_auto()
    log_action(datetime.now(), "CALIBRATE", "AUTO")


def resetPosition():
    n.move_joints([0, 0, 0, 0, 0, 0])
    log_action(datetime.now(), "MOVE_JOINTS", stringify([0, 0, 0, 0, 0, 0]))


def moveRobot(a1, a2, a3, a4, a5, a6):
    n.move_pose(a1, a2, a3, a4, a5, a6)
    log_action(datetime.now(), "MOVE_JOINTS", stringify(n.get_joints()))


def changeVelocity(speed):
    if speed < 1 or speed > 100:
        print("error setting up speed")
        return
    else:
        n.set_arm_max_velocity(speed)
        log_action(datetime.now(), "SET_ARM_MAX_VELOCITY", stringify(speed))


# Functions to handle how parameters change
def fetchBlock(a1, a2, a4, a5, a6, block_id, block_height, block_uncertainty):
    print("--- Slowing down the arm ---\n")

    height = (block_id * block_height) + ((0.5 * block_id + 0.5) * block_uncertainty)

    changeVelocity(90)
    moveRobot(a1, a2, height + 0.023, a4, a5, a6)

    changeVelocity(5)
    moveRobot(a1, a2, height, a4, a5, a6)
    moveRobot(a1, a2, height + 0.01, a4, a5, a6)


def dropBlock(blockNumber):
    print("--- Moving to Goal ---\n")

    if blockNumber == 1:
        changeVelocity(20)
        moveRobot(0.249, 0.0859, 0.143, 0, pi / 2, 0)
        # sleep
        moveRobot(0.249, 0.0859, 0.033, 0, pi / 2, 0)
        # sleep to let the block "drop"
        changeVelocity(90)
        moveRobot(0.249, 0.0859, 0.143, 0, pi / 2, 0)

    elif blockNumber == 2:
        changeVelocity(20)
        moveRobot(0.216, 0.0849, 0.11, 0, pi / 2, -pi / 2)
        # sleep
        moveRobot(0.216, 0.0849, 0.033, 0, pi / 2, -pi / 2)
        # sleep to let the block "drop"
        changeVelocity(90)
        moveRobot(0.216, 0.0849, 0.11, 0, pi / 2, -pi / 2)

    elif blockNumber == 3:
        changeVelocity(20)
        moveRobot(0.250, 0.0529, 0.077, 0, pi / 2, pi / 2)
        # sleep
        moveRobot(0.250, 0.0529, 0.033, 0, pi / 2, pi / 2)
        # sleep to let the block "drop"
        changeVelocity(90)
        moveRobot(0.250, 0.0529, 0.077, 0, pi / 2, pi / 2)

    else:
        changeVelocity(20)
        moveRobot(0.217, 0.0519, 0.044, 0, pi / 2, 0)
        # sleep
        moveRobot(0.217, 0.0519, 0.033, 0, pi / 2, 0)
        # sleep to let the block "drop"
        changeVelocity(90)
        moveRobot(0.217, 0.0519, 0.044, 0, pi / 2, 0)


# Functional block
try:
    params = load_params("params")

    print(" --- Starting Monte --- ")

    # TODO: handle logs
    print(" --- Starting to record log --- ")

    calibrateRobot()

    resetPosition()

    for block_id in range(1, params["blocks"] + 1)[::-1]:
        fetchBlock(
            params["origin"][0][0],
            params["origin"][0][1],
            params["origin"][1][0],
            params["origin"][1][1],
            params["origin"][1][2],
            block_id,
            params["block_height"],
            params["block_uncertainty"],
        )
        dropBlock(block_id)

    resetPosition()

    print(" --- Ended the trajectory intended --- \n")

    if raw_input("Type y to save the robot's log:") == "y":  # type: ignore
        with open("logs.txt", "w") as f:
            for action in actions:
                f.write(action + "\n")


except Exception as e:
    print(e)
