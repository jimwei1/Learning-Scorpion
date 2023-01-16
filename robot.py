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

        self.motors = {}

        self.robotID = p.loadURDF("body.urdf")

        pyrosim.Prepare_To_Simulate(self.robotID)

        self.Prepare_To_Sense()

    def Prepare_To_Sense(self):
            self.sensors = {}
            for linkName in pyrosim.linkNamesToIndices:
                #print(linkName)
                self.sensors[linkName] = SENSOR(linkName)
                
                self.values = numpy.zeros(c.timevalue)

    def Sense(self):
        for i in self.sensors:
            self.Get_Value(i)

        

    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        print("VALUES HERE")
        print(self.values)

                
