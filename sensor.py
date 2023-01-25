import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

class SENSOR:

    def __init__(self, linkName): #constructor

        self.SensorValues = numpy.zeros(c.timevalue)
        self.linkName = linkName
    
    def Get_Value(self, t):
        self.SensorValues[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)


        #print("VALUES HERE")
        #print(self.values)

    def SaveValues(self):
        numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/tempValuesforH.npy', self.sensorValues)