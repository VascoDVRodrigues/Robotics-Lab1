#!/usr/bin/env python


import sys
import yaml
import rospy  # type: ignore
import argparse

from datetime import datetime
from os.path import exists as file_exists
from niryo_one_python_api.niryo_one_api import *  # type: ignore
from niryo_one_msgs.msg._RobotState import *  # type: ignore
from geometry_msgs.msg._Point import *

sys.path.insert(1, "/home/vaco/catkin_ws/src/niryo_one_python_api")  # type: ignore

rospy.init_node("niryo_one_example_python_api")

pi = 3.14159
n = NiryoOne()  # type: ignore

actions = []
positions = []
start_time = datetime.now()

# Auxiliary functions


def get_cmd_line_params():
    parser = argparse.ArgumentParser(description="Tilling problem with Niryo One Robot")

    parser.add_argument("-f", "--file", metavar="FILE", type=str, help="Name of the YAML config file")

    parser.add_argument(
        "-l",
        "--log",
        action="store_true",
        help="Store a log file",
    )

    args = parser.parse_args()
    
    return vars(args)


def load_params(config_file):
    params = {}

    if not config_file:
        raise SystemExit("A file with the points must be specified")

    if not file_exists(config_file + ".yaml"):
        raise SystemExit("The specified config file does not exist")

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
    
    global n
    global positions

    pos = n.get_arm_pose().position
    positions.append(stringify([pos.x, pos.y, pos.z]))


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
def fetchBlock(pos, z):
    print("--- Fetching block ---\n")

    # hovers origin
    changeVelocity(90)
    moveRobot(pos["x"], pos["y"] + 0.035, z + 0.02, pos["roll"], pos["pitch"], pos["yaw"])
    moveRobot(pos["x"], pos["y"], z + 0.02, pos["roll"], pos["pitch"], pos["yaw"])

    # fetches the block
    changeVelocity(90)
    moveRobot(pos["x"], pos["y"], z, pos["roll"], pos["pitch"], pos["yaw"])

    # hovers origin with the block
    moveRobot(pos["x"], pos["y"], z + 0.02, pos["roll"], pos["pitch"], pos["yaw"])
    moveRobot(pos["x"], pos["y"] + 0.035, z + 0.02, pos["roll"], pos["pitch"], pos["yaw"])


def dropBlock(pos, z):
    print("--- Dropping block ---\n")

    # hovers target area with the block
    moveRobot(pos["x"], pos["y"], 2 * z + 0.02, pos["roll"], pos["pitch"], pos["yaw"])

    # drops the block
    moveRobot(pos["x"], pos["y"], z, pos["roll"], pos["pitch"], pos["yaw"])

    # hovers target area with the block
    moveRobot(pos["x"], pos["y"], z + 0.02, pos["roll"], pos["pitch"], pos["yaw"])


# Functional block
try:
    cmd_params = get_cmd_line_params()
    
    params = load_params(cmd_params["file"])

    print(" --- Starting to move blocks --- \n")

    # TODO: handle logs
    print(" --- Starting to log actions --- \n")

    calibrateRobot()
    resetPosition()

    for block_id in range(1, params["blocks"] + 1)[::-1]:
        initial_z = (block_id * params["block_height"]) + ((0.5 * block_id + 0.5) * params["block_uncertainty"])
        target_z = params["block_height"]

        fetchBlock(params["origin"], initial_z)

        dropBlock(params["goal" + str(block_id)], target_z)

    resetPosition()

    print(" --- Moved all the blocks --- \n")

    if cmd_params["log"]:  # type: ignore
        with open("logs.txt", "w") as f:
            for action in actions:
                f.write(action + "\n")
                
        with open("positions.txt", "w") as f:
            for position in positions:
                f.write(position + "\n")


except Exception as e:
    print(e)
