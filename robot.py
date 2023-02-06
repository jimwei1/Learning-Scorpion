import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c
import os as os

from pyrosim.neuralNetwork import NEURAL_NETWORK

from sensor import SENSOR
from motor import MOTOR

class ROBOT: #class name

    def __init__(self, solutionID): #constructor

        self.solutionID = solutionID

        self.robotID = p.loadURDF("body.urdf")

        #self.ballID = p.loadURDF("ball.urdf")

        self.motors = {}

        self.sensors = {}

        pyrosim.Prepare_To_Simulate(self.robotID)

        #pyrosim.Prepare_To_Simulate(self.ballID)

        self.Prepare_To_Sense()

        self.Prepare_To_Act()

        #brainID = "brain" + solutionID + ".nndf"

        self.nn = NEURAL_NETWORK("brain" + solutionID + ".nndf")

        os.system("rm " + "brain" + solutionID + ".nndf")

    def Prepare_To_Sense(self):
    
            for linkName in pyrosim.linkNamesToIndices:

                self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for i in self.sensors:
            self.sensors[i].Get_Value(t)


    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)
    
    
    def Act(self, t):
        for neuronName in self.nn.Get_Neuron_Names():
            
            if self.nn.Is_Motor_Neuron(neuronName):
                
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)

                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange

                self.motors[jointName].Set_Value(self.robotID, desiredAngle)
                

    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):

        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotID)
        basePosition = basePositionAndOrientation[0]
        xCoordinateofLinkZero = basePosition[0]


        print("GET_FITNESS:")
        print("tmp" + self.solutionID + ".txt", "w")
        f = open("tmp" + self.solutionID + ".txt", "w")

        os.system("mv tmp" + str(self.solutionID) + ".txt fitness" + str(self.solutionID) + ".txt")
        f.write(str(xCoordinateofLinkZero))
        f.close()
        

                
