import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

class SENSOR:

    def __init__(self): #constructor

        self.backLegSensorValues = numpy.zeros(c.timevalue)
        self.frontLegSensorValues = numpy.zeros(c.timevalue)