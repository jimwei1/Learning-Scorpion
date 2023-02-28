import numpy as np
import pyrosim.pyrosim as pyrosim
#from robot import ROBOT
import random as random
import os as os
import time as time
import constants as c
#import pybullet as pybullet

class SOLUTION:

    def __init__(self, nextAvailableID): #constructor
        
        self.myID = nextAvailableID

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        os.system("python3 simulate.py " + directOrGUI + " " + str(self.myID))

    def Wait_For_Simulation_To_End(self):
        fileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fileName):
            time.sleep(0.01)
        f = open(fileName, "r")
        self.fitness = float(f.read())
        f.close()
        os.system("rm " + fileName)


    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()


    def Create_Body(self):
        bodyID = "body" + str(self.myID) + ".urdf"
        pyrosim.Start_URDF(bodyID)

        jointAxisConstant = "1 1 0"

        #Random number of links
        self.numofLinks = random.randint(13, 15)

        #Random Sizes Dictionary
        self.linkSizeConstants = dict.fromkeys(range(self.numofLinks), None)

        for i in range(self.numofLinks):
            self.linkSizeConstants[i] = [random.random(), random.random(), random.random()]


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
        self.LinkPositions = dict.fromkeys(range(self.numofLinks), None)

        for i in self.LinkPositions:
            #yPos = int(linkSizeConstants[i + 1][1]) // 2
            yPos = 0.25
            self.LinkPositions[i] = [0, yPos, 0]

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
            colorID[i] = '<color rgba="0 1.0 1.0 1.0"/>'

        for i in range(len(self.randomLink)):
            linkNum = str(self.randomLink[i])[4]
            intlinkNum = int(linkNum)

            colorID[intlinkNum] = '<color rgba="0 1.0 0 1.0"/>'

        colorName = dict.fromkeys(range(self.numofLinks), None)

        for i in colorName:
            if colorID[i] == '<color rgba="0 1.0 1.0 1.0"/>':
                colorName[i] = '<material name="Cyan">'
            
            if colorID[i] == '<color rgba="0 1.0 0 1.0"/>':
                colorName[i] = '<material name="Green">'
            

        pyrosim.Send_Cube(name="Torso", pos=[0, 2, 2] , size=[0.5,0.5,0.5])
        pyrosim.Send_Joint(name = "Torso_Link0" , parent= "Torso" , child = "Link0" , type = "revolute", position = [0, 2.25, 2], jointAxis = jointAxisConstant)
        for i in range(self.numofLinks):
            
            pyrosim.Send_Cube(name=self.linkNames[i], pos=self.LinkPositions[i] , size=self.linkSizeConstants[i], colorName = colorName[i], colorID = colorID[i])
            #print("SENDING CUBE:")
            #print("name: " + str(self.linkNames[i]) + " size: "+ str(self.linkSizeConstants[i]))

        legJointAxis = "1 0 0"
        for randLink in range(len(self.randomLink)):
            randomLink = self.randomLink[randLink]
            self.LeftjointName = str(randomLink) + "_LeftLeg" + str(randomLink)
            self.LeftchildName = "LeftLeg" + str(randomLink)
            pyrosim.Send_Joint( name = self.LeftjointName, parent= randomLink , child = self.LeftchildName , type = "revolute", position = [-0.25, 0, 0], jointAxis = jointAxisConstant)

            pyrosim.Send_Cube(name=self.LeftchildName, pos=[-0.5, 0, 0] , size=[1, 0.2, 0.2])
            self.LeftlowerJointName = "LeftLeg" + str(randomLink) + "_LeftLowerLeg" + str(randomLink)
            self.LeftlowerChildName = "LeftLowerLeg" + str(randomLink)
            pyrosim.Send_Joint( name = self.LeftlowerJointName, parent= self.LeftchildName , child = self.LeftlowerChildName , type = "revolute", position = [-1, 0, 0], jointAxis = legJointAxis)

            pyrosim.Send_Cube(name=self.LeftlowerChildName, pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        
        for randLink in range(len(self.randomLink)):
            randomLink = self.randomLink[randLink]
            self.RightjointName = str(randomLink) + "_RightLeg" + str(randomLink)
            self.RightchildName = "RightLeg" + str(randomLink)
            pyrosim.Send_Joint( name = self.RightjointName, parent= randomLink , child = self.RightchildName , type = "revolute", position = [0.25, 0, 0], jointAxis = jointAxisConstant)

            pyrosim.Send_Cube(name=self.RightchildName, pos=[0.5, 0, 0] , size=[1, 0.2, 0.2])
            self.RightlowerJointName = "RightLeg" + str(randomLink) + "_RightLowerLeg" + str(randomLink)
            self.RightlowerChildName = "RightLowerLeg" + str(randomLink)
            pyrosim.Send_Joint( name = self.RightlowerJointName, parent= self.RightchildName , child = self.RightlowerChildName , type = "revolute", position = [1, 0, 0], jointAxis = legJointAxis)

            pyrosim.Send_Cube(name=self.RightlowerChildName, pos= [0, 0, -0.5] , size=[0.2, 0.2, 1])

        for i in range(self.numofLinks - 1):
            if i < 9:
                pyrosim.Send_Joint(name = self.jointNames[i] , parent = self.jointNames[i][0:5] , child = self.jointNames[i][6:11] , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i<9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:11]))

            if i == 9:
                pyrosim.Send_Joint(name = self.jointNames[i] , parent = str(self.jointNames[i][0:5]) , child = str(self.jointNames[i][6:12]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i=9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:12]))
            if i > 9:
                pyrosim.Send_Joint(name = self.jointNames[i] , parent = str(self.jointNames[i][0:6]) , child = str(self.jointNames[i][7:13]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i>9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:6]) + " | " + str(self.jointNames[i][7:13]))

        pyrosim.End()

    def Create_Brain(self):

        brainID = "brain" + str(self.myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)
    
        pyrosim.Start_NeuralNetwork(brainID)

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = self.randomLink[0])
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = self.randomLink[1])
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = self.randomLink[2])
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = self.randomLink[3])
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = self.LeftchildName)
        pyrosim.Send_Sensor_Neuron(name = 5, linkName = self.LeftlowerChildName)
        pyrosim.Send_Sensor_Neuron(name = 6, linkName = self.RightchildName)
        pyrosim.Send_Sensor_Neuron(name = 7, linkName = self.RightlowerChildName)

        randomJoint1 = str(self.randomLink[0]) + "_Link" + str(int(self.randomLink[0][4]) + 1)
        randomJoint2 = str(self.randomLink[1]) + "_Link" + str(int(self.randomLink[1][4]) + 1)
        randomJoint3 = str(self.randomLink[2]) + "_Link" + str(int(self.randomLink[2][4]) + 1)
        randomJoint4 = str(self.randomLink[3]) + "_Link" + str(int(self.randomLink[3][4]) + 1)

        pyrosim.Send_Motor_Neuron(name = 8 , jointName = randomJoint1)
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = randomJoint2)
        pyrosim.Send_Motor_Neuron(name = 10 , jointName = randomJoint3)
        pyrosim.Send_Motor_Neuron(name = 11 , jointName = randomJoint4)
        pyrosim.Send_Motor_Neuron(name = 12 , jointName = self.LeftjointName)
        pyrosim.Send_Motor_Neuron(name = 13 , jointName = self.LeftlowerJointName)
        pyrosim.Send_Motor_Neuron(name = 14 , jointName = self.RightjointName)
        pyrosim.Send_Motor_Neuron(name = 15 , jointName = self.RightlowerJointName)


        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = (self.weights * 2) - 1
        for currentRow in range(c.numSensorNeurons - 1):
            for currentColumn in range(c.numMotorNeurons - 1):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow , targetNeuronName = (currentColumn + c.numSensorNeurons - 1), weight = self.weights[currentRow][currentColumn])

        pyrosim.End()


    def Mutate(self):
        self.Mutate_Body()
        self.Mutate_Brain()
    
    def Mutate_Body(self):
        # for leg in range(self.numofLegs):
        #     tempNum = int(self.numofLinksDict[leg])
        #     #print(tempNum)
        #     randLink = random.randint(0, tempNum)
        #     self.linkSizeConstantsDict[leg][randLink] = [random.random(), random.random(), random.random()]
        randLink = random.randint(0, self.numofLinks)
        maxLink = self.numofLinks
        self.linkSizeConstants[maxLink] = [random.random(), random.random(), random.random()]

    def Mutate_Brain(self):
        # for leg in range(self.numofLegs):
        #     tempLinkNum = len(self.weights[leg]) - 1
        #     randLink = random.randint(0, tempLinkNum)
        #     tempJointNum = len(self.weights[leg][randLink]) - 1
        #     randJoint = random.randint(0, tempJointNum)

        #     self.weights[leg][randLink][randJoint]= int(random.random() * 2 - 1)
        randLink = random.randint(0, c.numSensorNeurons - 1)
        randJoint = random.randint(0, c.numMotorNeurons - 1)
        self.weights[randLink][randJoint]= int(random.random() * 2 - 1)

    
    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID



