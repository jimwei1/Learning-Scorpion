import numpy as numpy
import pyrosim.pyrosim as pyrosim
from robot import ROBOT
import random as random
import os as os
import time as time

class SOLUTION:

    def __init__(self, nextAvailableID): #constructor
        
        self._myID = nextAvailableID
        self.weights = numpy.random.rand(3, 2)
        self.weights = (self.weights * 2) - 1

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

        os.system("python3 simulate.py " + directOrGUI + " " + str(self._myID) + " &")


    def Wait_For_Simulation_To_End(self):

        fitnessFileName = "fitness" + str(self._myID) + ".txt"

        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)

        print("SOLUTION FITNESSFILENAME")
        print(fitnessFileName)
        
        f = open(fitnessFileName, "r")

        self.fitness = float(f.read())
        f.close()

        os.system("del " + fitnessFileName)

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[1,1,1])
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
        
        for currentRow in [0,1,2]:
            for currentColumn in [0,1]:
                pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = (currentColumn + 3), weight = self.weights[currentRow][currentColumn])
        
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0 , 2)
        randomColumn = random.randint(0, 1)

        self.weights[randomRow][randomColumn] = (random.random() * 2) - 1



