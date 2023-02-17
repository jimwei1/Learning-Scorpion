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
        self.numofLegs = random.randint(2,5)

        self.numofLinksDict = dict.fromkeys(range(self.numofLegs), None)

        self.linkSizeConstantsDict = []

        self.linkNamesDict = []

        self.jointNamesDict = []

        self.linkPositionsDict = []

        self.randomLinkDict = []

        self.colorIdDict = []

        self.colorNameDict = []

        for i in range(self.numofLegs):
            self.numofLinksDict[i] = random.randint(8, 12)

            #Link Size Constants Dictionary, in the i location of array self.LinkSizeConstants
            self.linkSizeConstantsDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            #Link Size Constants Array input, in the Link Size Constants Dictionary. I'm confused.
            for x in range(self.numofLinksDict[i]):
                self.linkSizeConstantsDict[i][x] = [random.random(), random.random(), random.random()]

            #Link Names Dictionary, in the i location of array self.linkNamesDict
            self.linkNamesDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            #Creating Link Names for each leg
            for x in range(self.numofLinksDict[i]):
                self.linkNamesDict[i][x] = str(i) + "Link" + str(x)

            #Joint Names Dictionary, in the i location of array self.jointNamesDict
            self.jointNamesDict[i] = dict.fromkeys(range(self.numofLinksDict[i] - 1), None)

            #Creating Joint Names for each leg
            for x in range(self.numofLinksDict[i] - 1):
                self.jointNamesDict[i][x] = str(i) + "Link" + str(x) + "_" + str(i) + "Link" + str(x+1)
            
            #Link Positions Dictionary, in the i location of array self.linkPositionsDict
            self.linkPositionsDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            #Setting Link Positions based on Leg Number
            for x in self.linkPositionsDict[i]:
                
                #Neg z Leg
                if i == 0:
                    xPos = 0
                    yPos = 0
                    zPos = -1 *  self.linkSizeConstantsDict[i][x][2] // 2
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

                #Neg x Leg
                if i == 1:
                    xPos = -1 *  self.linkSizeConstantsDict[i][x][0] // 2  
                    yPos = 0
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

                #Pos z Leg
                if i == 2:
                    xPos = 0
                    yPos = 0
                    zPos = self.linkSizeConstantsDict[i][x][2] // 2
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

                #Pos x Leg
                if i == 3:
                    xPos = self.linkSizeConstantsDict[i][x][0] // 2  
                    yPos = 0
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]
                
                #Neg y Leg
                if i == 4:
                    xPos = 0
                    yPos = -1 * self.linkSizeConstantsDict[i][x][0] // 2  
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]
                    
                #Pos y Leg
                if i == 5:
                    xPos = 0
                    yPos = self.linkSizeConstantsDict[i][x][0] // 2  
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]
                
            #4 Random Links Generated, saved in location i of self.randomLinkDict
            self.randomLinkDict[i] = dict.fromkeys(range(4), None)

            #Make sure each Random Link is unique
            for x in self.randomLinkDict[i]:
                while True:
                    self.randomLinkDict[i][x] = str(i) + "Link" + str(random.randint(1, 11))
                    unique = True

                    for a in range(i):
                        if self.randomLinkDict[i][x] == self.randomLinkDict[i][a]:
                            unique = False

                    if unique == True:
                        break

            #Generate Color IDs, saved in self.colorIdDict[i]
            self.colorIdDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            #Fill in Color IDs
            for x in self.colorIdDict[i]:
                self.colorIdDict[i][x] = '<color rgba="0 0 1.0 1.0"/>'
            
            for x in range(len(self.randomLinkDict[i])):
                linkNum = int(''.join(filter(str.isdigit, self.randomLinkDict[i][x])))

                self.colorIdDict[i][linkNum] = '<color rgba="0 1.0 0 1.0"/>'

            #Fill in ColorNames
            self.colorNameDict[i] = dict.fromkeys(range(self.numofLinks), None)

            for x in self.colorNameDict[i]:

                if self.colorIdDict[i][x] == '<color rgba="0 0 1.0 1.0"/>':
                    self.colorNameDict[i][x] = '<material name="Blue">'
                
                if self.colorIdDict[i][x] == '<color rgba="0 1.0 0 1.0"/>':
                    self.colorNameDict[i][x] = '<material name="Green">'
                

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        jointAxisConstant = "1 1 0"

        self.Create_Leg_Dictionaries()
        
        startPos = [0, 2, 10]

        pyrosim.Send_Cube(name="Torso", pos = startPos , size=[0.5,0.5,0.5], colorName = '<material name="Red">', colorID = '<color rgba="1.0 0 0 1.0"/>')
       
       #First (# of Legs) Cubes. Basically creating a base to generate the rest of each leg.
        for i in range(self.numofLegs):
            jointPos = numpy.add(startPos, self.linkPositionsDict[i][0])
            firstChildName = "Base" + str(i)
            firstJointName = "Torso_" + firstChildName

            pyrosim.Send_Joint(name = firstJointName , parent= "Torso" , child = firstChildName , type = "revolute", position = jointPos, jointAxis = jointAxisConstant)




        
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



