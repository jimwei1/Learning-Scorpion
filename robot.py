import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c
import os as os
import math as math

from pyrosim.neuralNetwork import NEURAL_NETWORK

from sensor import SENSOR
from motor import MOTOR
#from simulate import SIULATION

class ROBOT:
    def __init__(self, solutionID, world):
        self.world = world
        self.robotId = p.loadURDF("body.urdf")
        self.solutionID = solutionID
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain" + self.solutionID + ".nndf")
        os.system("rm brain" + self.solutionID + ".nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            #print("SELF.MOTORS: ")
            #print(self.motors)
            #if jointName in self.motors:
            self.motors[jointName] = MOTOR(jointName)

    def Sense(self, element):
        for sensorInstance in self.sensors.values():
            sensorInstance.Get_Value(element)

    def Act(self, element):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                #print("ACT DESIRED ANGLE: ")
                #print(self.nn.Get_Value_Of(neuronName))
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                #self.motors[bytes(jointName, encoding='utf-8')].Set_Value(self.robotId, desiredAngle)
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)

    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId, 0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]

        f = open("tmp" + str(self.solutionID) + ".txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.system("mv tmp" + str(self.solutionID) + ".txt fitness" + str(self.solutionID) + ".txt")
        
        

                
