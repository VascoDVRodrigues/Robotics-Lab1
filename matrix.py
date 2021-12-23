#!/usr/bin/bash
from niryo_one_python.api.niryo_one_api import *
import rospy
import time
import math

n = NiryoOne()
n.calibrate()

joints = n.get_joints()

class Robot:
    def __init__(self, name, joints):
        self.name = name
        self.theta = joints
        self.dab = 103
        self.dbc = 80
        self.dcd = 210
    
    def calculate_transformation_matrices(self):
        Tba = [[math.cos(self.theta[0]),math.sin(self.theta[0]),0,0],
            [-1*math.sin(self.theta[0]),math.cos(self.theta[0]),0,0],
            [0,0,1,self.dab],
            [0,0,0,1]]

        Tbc = [[1,0,0,0],
            [0, math.cos(self.theta[1]), -1 * math.sin(self.theta[1]), self.dbc * math.cos(self.theta[1])],
            [0,math.sin(self.theta[1]), math.cos(self.theta[1]), self.dbc * math.sin(self.theta[1])],
            [0,0,0,1]]



robot = Robot("alberto",joints)
robot.calculate_transformation_matrices()

