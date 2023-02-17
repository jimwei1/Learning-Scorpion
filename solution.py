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
            print("stuck sleeping")
        
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

    def Create_Leg_Dictionaries(self):
        self.numofLegs = random.randint(3,6)

        self.numofLinksDict = dict.fromkeys(range(self.numofLegs), None)

        self.linkSizeConstantsDict = []

        self.linkNamesDict = []

        self.jointNamesDict = []

        self.linkPositionsDict = []

        for i in range(self.numofLegs):
            self.numofLinksDict[i] = random.randint(8, 12)

            #Link Size Constants Dictionary, in the i location of array self.LinkSizeConstants
            self.linkSizeConstantsDict[i] = dict.fromkeys(range(self.numofLinks), None)

            #Link Size Constants Array input, in the Link Size Constants Dictionary. I'm confused.
            for x in range(self.numofLinksDict[i]):
                self.linkSizeConstantsDict[i][x] = [random.random(), random.random(), random.random()]

            #Link Names Dictionary, in the i location of array self.linkNamesDict
            self.linkNamesDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            #Creating Link Names for each leg
            for x in range(self.numofLinks):
                self.linkNamesDict[i][x] = str(i) + "Link" + str(x)

            #Joint Names Dictionary, in the i location of array self.jointNamesDict
            self.jointNamesDict[i] = dict.fromkeys(range(self.numofLinksDict[i] - 1), None)

            #Creating Joint Names for each leg
            for x in range(self.numofLinksDict[i] - 1):
                self.jointNamesDict[i][x] = str(i) + "Link" + str(x) + "_" + str(i) + "Link" + str(x+1)
            
            #Link Positions Dictionary, in the i location of array self.linkPositionsDict
            self.linkPositionsDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            
                






    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        jointAxisConstant = "1 1 0"



        #Link Position Dictionary
        self.leftLinkPositions = dict.fromkeys(range(self.numofLinks), None)

        for i in self.leftLinkPositions:
            #yPos = int(linkSizeConstants[i + 1][1]) // 2
            yPos = 0.25
            zPos = -0.25
            self.leftLinkPositions[i] = [0, yPos, zPos]

        #Right Position Dictionary
        self.rightLinkPositions = dict.fromkeys(range(self.numofLinks), None)

        for i in self.rightLinkPositions:
            #yPos = int(linkSizeConstants[i + 1][1]) // 2
            yPos = -0.25
            zPos = -0.25
            self.rightLinkPositions[i] = [0, yPos, zPos]

        #Left Randomly Selected Link Dictionary
        self.leftRandomLink = dict.fromkeys(range(4), None)

        #Makes sure each Left Random Link is unique
        for i in self.leftRandomLink:
            while True:
                self.leftRandomLink[i] = "LeftLink" + str(random.randint(1, 7))
                unique = True

                for a in range(i):
                    if self.leftRandomLink[i] == self.leftRandomLink[a]:
                        unique = False

                if unique == True:
                    break
            
        #Right Randomly Selected Link Dictionary
        self.rightRandomLink = dict.fromkeys(range(4), None)

        #Makes sure each Right Random Link is unique
        for i in self.rightRandomLink:
            while True:
                self.rightRandomLink[i] = "RightLink" + str(random.randint(1, 7))
                unique = True

                for a in range(i):
                    if self.rightRandomLink[i] == self.rightRandomLink[a]:
                        unique = False

                if unique == True:
                    break

                    
        #Left ColorID Dictionary
        leftColorID = dict.fromkeys(range(self.numofLinks), None)

        for i in leftColorID:
            leftColorID[i] = '<color rgba="0 0 1.0 1.0"/>'

        for i in range(len(self.leftRandomLink)):
            linkNum = int(''.join(filter(str.isdigit, self.leftRandomLink[i])))
            intlinkNum = int(linkNum)

            leftColorID[intlinkNum] = '<color rgba="0 1.0 0 1.0"/>'

        #Right ColorID Dictionary
        rightColorID = dict.fromkeys(range(self.numofLinks), None)

        for i in rightColorID:
            rightColorID[i] = '<color rgba="0 0 1.0 1.0"/>'

        for i in range(len(self.rightRandomLink)):
            linkNum = int(''.join(filter(str.isdigit, self.rightRandomLink[i])))
            intlinkNum = int(linkNum)

            rightColorID[intlinkNum] = '<color rgba="0 1.0 0 1.0"/>'


        #Left ColorID --> ColorName
        leftColorName = dict.fromkeys(range(self.numofLinks), None)

        for i in leftColorName:
            if leftColorID[i] == '<color rgba="0 0 1.0 1.0"/>':
                leftColorName[i] = '<material name="Blue">'
            
            if leftColorID[i] == '<color rgba="0 1.0 0 1.0"/>':
                leftColorName[i] = '<material name="Green">'

        #Right ColorID --> ColorName
        rightColorName = dict.fromkeys(range(self.numofLinks), None)

        for i in rightColorName:
            if rightColorID[i] == '<color rgba="0 0 1.0 1.0"/>':
                rightColorName[i] = '<material name="Blue">'
            
            if rightColorID[i] == '<color rgba="0 1.0 0 1.0"/>':
                rightColorName[i] = '<material name="Green">'
            

        pyrosim.Send_Cube(name="Torso", pos=[0, 2, 10] , size=[0.5,0.5,0.5], colorName = '<material name="Blue">', colorID = '<color rgba="0 0 1.0 1.0"/>')
       
        pyrosim.Send_Joint(name = "Torso_Link0" , parent= "Torso" , child = "Link0" , type = "revolute", position = [0, 2.25, 4.75], jointAxis = jointAxisConstant)
        
        #Left Wing
        for i in range(self.numofLinks):
            
            pyrosim.Send_Cube(name=self.leftLinkNames[i], pos=self.leftLinkPositions[i] , size=linkSizeConstants[i], colorName = leftColorName[i], colorID = leftColorID[i])
            print("SENDING CUBE:")
            print("name: " + str(self.leftLinkNames[i]) + " size: "+ str(linkSizeConstants[i]))

        for i in range(self.numofLinks - 1):
            if i < 9:
                pyrosim.Send_Joint(name = self.leftJointNames[i] , parent = self.leftJointNames[i][0:9] , child = self.leftJointNames[i][11:20] , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i<9") LeftJoint1_LeftJoint2
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:11]))

            if i == 9:
                pyrosim.Send_Joint(name = self.leftJointNames[i] , parent = str(self.leftJointNames[i][0:9]) , child = str(self.leftJointNames[i][11:21]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i=9") LeftJoint9_LeftJoint10
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:12]))
            if i > 9:
                pyrosim.Send_Joint(name = self.leftJointNames[i] , parent = str(self.leftJointNames[i][0:10]) , child = str(self.leftJointNames[i][12:22]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i>9") LeftJoint10_LeftJoint11
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:6]) + " | " + str(self.jointNames[i][7:13]))

        #Right Wing
        for i in range(self.numofLinks):
            
            pyrosim.Send_Cube(name=self.rightLinkNames[i], pos=self.rightLinkPositions[i] , size=linkSizeConstants[i], colorName = rightColorName[i], colorID = rightColorID[i])
            print("SENDING CUBE:")
            print("name: " + str(self.leftLinkNames[i]) + " size: "+ str(linkSizeConstants[i]))

        for i in range(self.numofLinks - 1):
            if i < 9:
                pyrosim.Send_Joint(name = self.rightJointNames[i] , parent = self.rightJointNames[i][0:10] , child = self.rightJointNames[i][12:21] , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i<9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:11]))

            if i == 9:
                pyrosim.Send_Joint(name = self.rightJointNames[i] , parent = str(self.rightJointNames[i][0:10]) , child = str(self.rightJointNames[i][12:22]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i=9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:12]))
            if i > 9:
                pyrosim.Send_Joint(name = self.rightJointNames[i] , parent = str(self.rightJointNames[i][0:11]) , child = str(self.rightJointNames[i][13:23]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i>9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:6]) + " | " + str(self.jointNames[i][7:13]))
        pyrosim.End()


    def Create_Brain(self):

        brainID = "brain" + str(self._myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)
    
        pyrosim.Start_NeuralNetwork(brainID)

        #Left Neurons
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = self.leftRandomLink[0])
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = self.leftRandomLink[1])
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = self.leftRandomLink[2])
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = self.leftRandomLink[3])

        leftRandomJoint1 = str(self.leftRandomLink[0]) + "_LeftLink" + str(int(self.leftRandomLink[0][8]) + 1)
        leftRandomJoint2 = str(self.leftRandomLink[1]) + "_LeftLink" + str(int(self.leftRandomLink[1][8]) + 1)
        leftRandomJoint3 = str(self.leftRandomLink[2]) + "_LeftLink" + str(int(self.leftRandomLink[2][8]) + 1)
        leftRandomJoint4 = str(self.leftRandomLink[3]) + "_LeftLink" + str(int(self.leftRandomLink[3][8]) + 1)


        print("LEFT RANDOM JOINTS:")
        print(leftRandomJoint1)
        print(leftRandomJoint2)
        print(leftRandomJoint3)
        print(leftRandomJoint4)

        pyrosim.Send_Motor_Neuron(name = 4 , jointName = leftRandomJoint1)
        pyrosim.Send_Motor_Neuron(name = 5 , jointName = leftRandomJoint2)
        pyrosim.Send_Motor_Neuron(name = 6 , jointName = leftRandomJoint3)
        pyrosim.Send_Motor_Neuron(name = 7 , jointName = leftRandomJoint4)

        #Right Neurons
        pyrosim.Send_Sensor_Neuron(name = 8, linkName = self.rightRandomLink[0])
        pyrosim.Send_Sensor_Neuron(name = 9, linkName = self.rightRandomLink[1])
        pyrosim.Send_Sensor_Neuron(name = 10, linkName = self.rightRandomLink[2])
        pyrosim.Send_Sensor_Neuron(name = 11, linkName = self.rightRandomLink[3])

        rightRandomJoint1 = str(self.rightRandomLink[0]) + "_RightLink" + str(int(self.rightRandomLink[0][9]) + 1)
        rightRandomJoint2 = str(self.rightRandomLink[1]) + "_RightLink" + str(int(self.rightRandomLink[1][9]) + 1)
        rightRandomJoint3 = str(self.rightRandomLink[2]) + "_RightLink" + str(int(self.rightRandomLink[2][9]) + 1)
        rightRandomJoint4 = str(self.rightRandomLink[3]) + "_RightLink" + str(int(self.rightRandomLink[3][9]) + 1)


        print("RIGHT RANDOM JOINTS:")
        print(rightRandomJoint1)
        print(rightRandomJoint2)
        print(rightRandomJoint3)
        print(rightRandomJoint4)

        pyrosim.Send_Motor_Neuron(name = 12 , jointName = rightRandomJoint1)
        pyrosim.Send_Motor_Neuron(name = 13 , jointName = rightRandomJoint2)
        pyrosim.Send_Motor_Neuron(name = 14 , jointName = rightRandomJoint3)
        pyrosim.Send_Motor_Neuron(name = 15 , jointName = rightRandomJoint4)

    
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



