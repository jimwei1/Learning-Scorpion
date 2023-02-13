import numpy as numpy
import pyrosim.pyrosim as pyrosim
from robot import ROBOT
import random as random
import os as os
import time as time
import constants as c
#import pybullet as pybullet

class SOLUTION:

    def __init__(self, nextAvailableID): #constructor
        
        self._myID = nextAvailableID
        self.weights = numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = (self.weights * 2) - 1


    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        #self.Create_Ball()

        os.system("python3 simulate.py " + directOrGUI + " " + str(self._myID) + " 2&>1 &")

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self._myID) + ".txt"):
            time.sleep(0.01)
            #print("stuck sleeping")
        
        while os.path.exists("fitness" + str(self._myID) + ".txt"):
            
            fitnessFile = open("fitness" + str(self._myID) + ".txt", "r")
            fitnessContent = fitnessFile.read().strip('\n')
            if fitnessContent != "":
                self.fitness = float(fitnessContent)

                fitnessFile.close()
                break
            else:
                fitnessFile.close()

        os.system("rm " + "fitness" + str(self._myID) + ".txt")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        jointAxisConstant = "1 1 0"

        #Random number of links
        self.numofLinks = random.randint(8, 12)

        #Random Sizes Dictionary
        linkSizeConstants = dict.fromkeys(range(self.numofLinks), None)

        for i in range(self.numofLinks):
            linkSizeConstants[i] = [random.random(), random.random(), random.random()]

        #Link Names Dictionary
        self.linkNames = dict.fromkeys(range(self.numofLinks), None)

        for i in range(self.numofLinks):
            self.linkNames[i] = "Link" + str(i)

        #Joint Names Dictionary
        self.jointNames = dict.fromkeys(range(self.numofLinks - 1), None)

        for i in range(self.numofLinks - 1):
            self.jointNames[i] = "Link" + str(i) + "_Link" + str(i+1)

        #Constant Link Position
        LinkPositionConstant = [0, 0.50, 0]

        #Link Position Dictionary
        self.leftLinkPositions = dict.fromkeys(range(self.numofLinks), None)

        for i in self.leftLinkPositions:
            #yPos = int(linkSizeConstants[i + 1][1]) // 2
            yPos = 0.25
            zPos = -0.25
            self.leftLinkPositions[i] = [0, yPos, zPos]

        #Randomly Selected Link Dictionary
        self.randomLink = dict.fromkeys(range(4), None)

        #Makes sure each Random Link is unique
        for i in self.randomLink:
            while True:
                self.randomLink[i] = "Link" + str(random.randint(1, 7))
                unique = True

                for a in range(i):
                    if self.randomLink[i] == self.randomLink[a]:
                        unique = False

                if unique == True:
                    break
                    
        #ColorID Dictionary
        colorID = dict.fromkeys(range(self.numofLinks), None)

        for i in colorID:
            colorID[i] = '<color rgba="0 0 1.0 1.0"/>'

        for i in range(len(self.randomLink)):
            linkNum = str(self.randomLink[i])[4]
            intlinkNum = int(linkNum)

            colorID[intlinkNum] = '<color rgba="0 1.0 0 1.0"/>'

        colorName = dict.fromkeys(range(self.numofLinks), None)

        for i in colorName:
            if colorID[i] == '<color rgba="0 0 1.0 1.0"/>':
                colorName[i] = '<material name="Blue">'
            
            if colorID[i] == '<color rgba="0 1.0 0 1.0"/>':
                colorName[i] = '<material name="Green">'
            

        pyrosim.Send_Cube(name="Torso", pos=[0, 2, 5] , size=[0.5,0.5,0.5], colorName = '<material name="Blue">', colorID = '<color rgba="0 0 1.0 1.0"/>')
       
        pyrosim.Send_Joint(name = "Torso_Link0" , parent= "Torso" , child = "Link0" , type = "revolute", position = [0, 2.25, 4.75], jointAxis = jointAxisConstant)
        
        #Left
        for i in range(self.numofLinks):
            
            pyrosim.Send_Cube(name=self.linkNames[i], pos=self.leftLinkPositions[i] , size=linkSizeConstants[i], colorName = colorName[i], colorID = colorID[i])
            print("SENDING CUBE:")
            print("name: " + str(self.linkNames[i]) + " size: "+ str(linkSizeConstants[i]))

        for i in range(self.numofLinks - 1):
            if i < 9:
                pyrosim.Send_Joint(name = self.jointNames[i] , parent = self.jointNames[i][0:5] , child = self.jointNames[i][6:11] , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                print("Link i<9")
                print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:11]))

            if i == 9:
                pyrosim.Send_Joint(name = self.jointNames[i] , parent = str(self.jointNames[i][0:5]) , child = str(self.jointNames[i][6:12]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                print("Link i=9")
                print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:12]))
            if i > 9:
                pyrosim.Send_Joint(name = self.jointNames[i] , parent = str(self.jointNames[i][0:6]) , child = str(self.jointNames[i][7:13]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                print("Link i>9")
                print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:6]) + " | " + str(self.jointNames[i][7:13]))

        pyrosim.End()


    def Create_Brain(self):

        brainID = "brain" + str(self._myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)
    
        pyrosim.Start_NeuralNetwork(brainID)

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = self.randomLink[0])
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = self.randomLink[1])
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = self.randomLink[2])
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = self.randomLink[3])

        randomJoint1 = str(self.randomLink[0]) + "_Link" + str(int(self.randomLink[0][4]) + 1)
        randomJoint2 = str(self.randomLink[1]) + "_Link" + str(int(self.randomLink[1][4]) + 1)
        randomJoint3 = str(self.randomLink[2]) + "_Link" + str(int(self.randomLink[2][4]) + 1)
        randomJoint4 = str(self.randomLink[3]) + "_Link" + str(int(self.randomLink[3][4]) + 1)


        print("RANDOM JOINTS:")
        print(randomJoint1)
        print(randomJoint2)
        print(randomJoint3)
        print(randomJoint4)

        pyrosim.Send_Motor_Neuron(name = 4 , jointName = randomJoint1)
        pyrosim.Send_Motor_Neuron(name = 5 , jointName = randomJoint2)
        pyrosim.Send_Motor_Neuron(name = 6 , jointName = randomJoint3)
        pyrosim.Send_Motor_Neuron(name = 7 , jointName = randomJoint4)

    
        for currentRow in range(c.numSensorNeurons - 1):
            for currentColumn in range(c.numMotorNeurons - 1):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = (currentColumn + c.numSensorNeurons - 1), weight = self.weights[currentRow][currentColumn])

        pyrosim.End()


    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons - 1)

        self.weights[randomRow][randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self._myID = nextAvailableID



