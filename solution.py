import numpy as numpy
import pyrosim.pyrosim as pyrosim
from robot import ROBOT
import random as random
import os as os
import time as time
import constants as c

class SOLUTION:

    def __init__(self, nextAvailableID): #constructor
        
        self._myID = nextAvailableID
        self.weights = numpy.random.rand(3, 2)
        self.weights = (self.weights * 2) - 1


    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

        os.system("python3 simulate.py " + directOrGUI + " " + str(self._myID) + " 2&>1 &")
        


    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self._myID) + ".txt"):
            time.sleep(0.01)
        fitnessFile = open("fitness" + str(self._myID) + ".txt", "r")
        self.fitness = float(fitnessFile.read().strip('\n'))

        fitnessFile.close()

        os.system("rm " + "fitness" + str(self._myID) + ".txt")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1] , size=[1,1,1])

        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0, -0.5, 1], jointAxis = "1 0 0")

        pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0] , size=[0.2 , 1, 0.2])

        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0, 0.5, 1], jointAxis = "1 0 0")

        pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0] , size=[0.2, 1, 0.2])

        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5, 0, 1], jointAxis = "0 1 0")

        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0] , size=[1, 0.2, 0.2])

        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5, 0, 1], jointAxis = "0 1 0")

        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0] , size=[1, 0.2, 0.2])

        pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0, -1, 0], jointAxis = "1 0 0")

        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0, 1, 0], jointAxis = "1 0 0")

        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1, 0, 0], jointAxis = "0 1 0")

        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1, 0, 0], jointAxis = "0 1 0")

        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        pyrosim.End()


    def Create_Brain(self):

        brainID = "brain" + str(self._myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)

        pyrosim.Start_NeuralNetwork(brainID)
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

        pyrosim.Send_Synapse(sourceNeuronName = 0 , targetNeuronName = 3 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 1 , targetNeuronName = 3 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 0 , targetNeuronName = 4 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 1 , targetNeuronName = 4 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 2 , targetNeuronName = 4 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 2 , targetNeuronName = 3 , weight = 1)
        
        for currentRow in [c.numSensorNeurons - 1]:
            for currentColumn in [c.numMotorNeurons - 1]:
                pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = (currentColumn + c.numSensorNeurons), weight = self.weights[currentRow][currentColumn])
        
        pyrosim.End()


    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)

        self.weights[randomRow][randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self._myID = nextAvailableID



