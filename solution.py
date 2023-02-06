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
        self.weights = numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = (self.weights * 2) - 1


    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        self.Create_Ball()

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

        jointAxisConstant = "1 1 0"

        LinkSizeConstant1 = [0.5, 0.5, 0.5]
        LinkSizeConstant2 = [0.35, 0.35, 0.35]

        LinkPositionConstant = [0, 0.5, 0]

        pyrosim.Send_Sphere(name="Torso", pos=[0, 0, 0.5] , size=[0.5,0.5,0.5])

        pyrosim.Send_Joint( name = "Torso_Link1" , parent= "Torso" , child = "Link1" , type = "revolute", position = [0, 0, 0.25], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link1", pos=LinkPositionConstant , size=LinkSizeConstant1)

        pyrosim.Send_Joint( name = "Link1_Link2" , parent= "Link1" , child = "Link2" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link2", pos=LinkPositionConstant , size=LinkSizeConstant2)

        pyrosim.Send_Joint( name = "Link2_Link3" , parent= "Link2" , child = "Link3" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link3", pos=LinkPositionConstant , size=LinkSizeConstant1)

        pyrosim.Send_Joint( name = "Link3_Link4" , parent= "Link3" , child = "Link4" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link4", pos=LinkPositionConstant , size=LinkSizeConstant2)

        pyrosim.Send_Joint( name = "Link4_Link5" , parent= "Link4" , child = "Link5" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link5", pos=LinkPositionConstant , size=LinkSizeConstant1)

        pyrosim.Send_Joint( name = "Link5_Link6" , parent= "Link5" , child = "Link6" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link6", pos=LinkPositionConstant , size=LinkSizeConstant2)

        pyrosim.Send_Joint( name = "Link6_Link7" , parent= "Link6" , child = "Link7" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link7", pos=LinkPositionConstant , size=LinkSizeConstant1)  

        pyrosim.Send_Joint( name = "Link7_Link8" , parent= "Link7" , child = "Link8" , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)

        pyrosim.Send_Sphere(name="Link8", pos=LinkPositionConstant , size=LinkSizeConstant2)     

        pyrosim.End()


    def Create_Brain(self):

        brainID = "brain" + str(self._myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)

        pyrosim.Start_NeuralNetwork(brainID)
        #pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Link1")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "Link2")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "Link3")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "Link4")
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "Link5")
        pyrosim.Send_Sensor_Neuron(name = 5, linkName = "Link6")
        pyrosim.Send_Sensor_Neuron(name = 6, linkName = "Link7")
        pyrosim.Send_Sensor_Neuron(name = 7, linkName = "Link8")


        pyrosim.Send_Motor_Neuron(name = 8 , jointName = "Torso_Link1")
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = "Link1_Link2")
        pyrosim.Send_Motor_Neuron(name = 10 , jointName = "Link2_Link3")
        pyrosim.Send_Motor_Neuron(name = 11 , jointName = "Link3_Link4")
        pyrosim.Send_Motor_Neuron(name = 12 , jointName = "Link4_Link5")
        pyrosim.Send_Motor_Neuron(name = 13 , jointName = "Link5_Link6")
        pyrosim.Send_Motor_Neuron(name = 14 , jointName = "Link6_Link7")
        pyrosim.Send_Motor_Neuron(name = 15 , jointName = "Link7_Link8")

        #pyrosim.Send_Motor_Neuron(name = 9 , jointName = "Torso_BackLeg")
        #pyrosim.Send_Motor_Neuron(name = 10 , jointName = "Torso_LeftLeg")
        #pyrosim.Send_Motor_Neuron(name = 11 , jointName = "Torso_RightLeg")
        
        for currentRow in range(c.numSensorNeurons - 1):
            for currentColumn in range(c.numMotorNeurons - 1):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = (currentColumn + c.numSensorNeurons - 1), weight = self.weights[currentRow][currentColumn])
        pyrosim.End()
    
    def Create_Ball(self):
        pyrosim.Start_URDF("ball.urdf")
        pyrosim.Send_Cube(name="tempBall", pos=[0, -0.5, 0.5] , size=[0.5,0.5,0.5])
        pyrosim.End()


    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)

        self.weights[randomRow][randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self._myID = nextAvailableID



