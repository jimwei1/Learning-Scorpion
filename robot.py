import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

from sensor import SENSOR
from motor import MOTOR

class ROBOT: #class name

    def __init__(self): #constructor

        self.sensors = {}
        self.motors = {}

        self.robotID = p.loadURDF("body.urdf")

        pyrosim.Prepare_To_Simulate(self.robotID)