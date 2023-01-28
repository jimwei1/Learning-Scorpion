import numpy as numpy
import pyrosim.pyrosim as pyrosim
from robot import ROBOT
import random as random
import os as os

class SOLUTION:

    def __init__(self): #constructor
        
        self.weights = numpy.random.rand(3, 2)
        self.weights = self.weights * 2 - 1

    #L Step 21

    def Evaluate(self):
        os.system("python3 simulate.py")

    def Create_World():
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()

    def Create_Body():
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[1,1,1])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[1,1,1])
        pyrosim.End()

    #THIS SELF DOWN HERE MIGHT CAUSE PROBLEMS. NEEDED TO ADD IN L STEP 24.
    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain.nndf")
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

        pyrosim.Send_Synapse(sourceNeuronName = 0 , targetNeuronName = 3 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 1 , targetNeuronName = 3 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 0 , targetNeuronName = 4 , weight = 1)
        pyrosim.Send_Synapse(sourceNeuronName = 0 , targetNeuronName = 4 , weight = 1)
        
        #Random Search
        for currentRow in [0,1,2]:
            for currentColumn in [0,1]:
                pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = currentColumn + 3, weight = self.weights[currentRow][currentColumn])
        
        pyrosim.End()




