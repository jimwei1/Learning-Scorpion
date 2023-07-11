from sensor import SENSOR
from motor import MOTOR
import pyrosim.pyrosim as pyrosim
import pybullet as p
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c
import numpy as np

class ROBOT:

    def __init__(self, solutionID):
        bodyID = "body" + solutionID +  ".urdf"
        brainID = "brain" + solutionID + ".nndf"
        self.robotId = p.loadURDF(bodyID)
        self.nn = NEURAL_NETWORK(brainID)
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        # os.system("rm " + brainID)
        # os.system("rm " + bodyID)
        os.system("del " + brainID)
        os.system("del " + bodyID)

    # generate the sensors
    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    # tell each sensor to check its value
    def Sense(self, step):
        counter = 0
        for sensor in self.sensors.values():
            sensor.Get_Value(step, counter) # counter to tell the sensor function what sensor it is
            counter += 1

    # generate the motors
    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    # move joints based on motor neuron values
    def Act(self, step):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)
                
    def Think(self):
        self.nn.Update()

    def Get_Fitness(self, solutionID):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)

        basePosition = basePositionAndOrientation[0]

        xPosition = basePosition[0]

        # print("GET_FITNESS:")
        # print("tmp" + solutionID + ".txt", "w")
        # f = open("tmp" + solutionID + ".txt", "w")

        # os.system("mv tmp" + str(solutionID) + ".txt fitness" + str(solutionID) + ".txt")
        # f.write(str(xPosition))
        # f.close()
        print("GET_FITNESS:")
        print("tmp" + solutionID + ".txt", "w")
        f = open("tmp" + solutionID + ".txt", "w")

        os.system("move tmp" + str(solutionID) + ".txt fitness" + str(solutionID) + ".txt")
        f.write(str(xPosition))
        f.close()

