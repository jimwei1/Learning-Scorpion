import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

from pyrosim.neuralNetwork import NEURAL_NETWORK

from sensor import SENSOR
from motor import MOTOR

class ROBOT: #class name

    def __init__(self): #constructor

        self.robotID = p.loadURDF("body.urdf")

        self.motors = {}

        pyrosim.Prepare_To_Simulate(self.robotID)

        self.Prepare_To_Sense()

        self.nn = NEURAL_NETWORK("brain.nndf")

    def Prepare_To_Sense(self):
            self.sensors = {}
            for linkName in pyrosim.linkNamesToIndices:

                self.sensors[linkName] = SENSOR(linkName)
    
                self.values = numpy.zeros(c.timevalue)

    def Sense(self, t):
        for i in self.sensors:
            self.sensors[i].Get_Value(t)


    def Prepare_To_Act(self):

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = SENSOR(jointName)
                
            self.values = numpy.zeros(c.timevalue)
    
    def Act(self):
        for i in self.motors:
            self.Set_Value(i)

    def Think(self):
        self.nn.Print()

                
