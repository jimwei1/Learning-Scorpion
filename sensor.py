import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

class SENSOR:

    def __init__(self, linkname): #constructor

        self.backLegSensorValues = numpy.zeros(c.timevalue)
        self.frontLegSensorValues = numpy.zeros(c.timevalue)
        self.linkname = linkname
    
    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

        #print("VALUES HERE")
        #print(self.values)