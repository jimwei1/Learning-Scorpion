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

        #Left Link Names Dictionary
        self.leftLinkNames = dict.fromkeys(range(self.numofLinks), None)

        for i in range(self.numofLinks):
            self.leftLinkNames[i] = "LeftLink" + str(i)

        #Right Link Names Dictionary
        self.rightLinkNames = dict.fromkeys(range(self.numofLinks), None)

        for i in range(self.numofLinks):
            self.rightLinkNames[i] = "RightLink" + str(i)

        #Left Joint Names Dictionary
        self.leftJointNames = dict.fromkeys(range(self.numofLinks - 1), None)

        for i in range(self.numofLinks - 1):
            self.leftJointNames[i] = "LeftLink" + str(i) + "_LeftLink" + str(i+1)

        #Right Joint Names Dictionary
        self.rightJointNames = dict.fromkeys(range(self.numofLinks - 1), None)

        for i in range(self.numofLinks - 1):
            self.rightJointNames[i] = "RightLink" + str(i) + "_RightLink" + str(i+1)


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
            linkNum = str(self.leftRandomLink[i])[4]
            intlinkNum = int(linkNum)

            leftColorID[intlinkNum] = '<color rgba="0 1.0 0 1.0"/>'

        #Right ColorID Dictionary
        rightColorID = dict.fromkeys(range(self.numofLinks), None)

        for i in rightColorID:
            rightColorID[i] = '<color rgba="0 0 1.0 1.0"/>'

        for i in range(len(self.rightRandomLink)):
            linkNum = str(self.rightRandomLink[i])[4]
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
            

        pyrosim.Send_Cube(name="Torso", pos=[0, 2, 5] , size=[0.5,0.5,0.5], colorName = '<material name="Blue">', colorID = '<color rgba="0 0 1.0 1.0"/>')
       
        pyrosim.Send_Joint(name = "Torso_Link0" , parent= "Torso" , child = "Link0" , type = "revolute", position = [0, 2.25, 4.75], jointAxis = jointAxisConstant)
        
        #Left Wing
        for i in range(self.numofLinks):
            
            pyrosim.Send_Cube(name=self.leftLinkNames[i], pos=self.leftLinkPositions[i] , size=linkSizeConstants[i], colorName = leftColorName[i], colorID = leftColorID[i])
            print("SENDING CUBE:")
            print("name: " + str(self.leftLinkNames[i]) + " size: "+ str(linkSizeConstants[i]))

        for i in range(self.numofLinks - 1):
            if i < 9:
                pyrosim.Send_Joint(name = self.leftJointNames[i] , parent = self.leftJointNames[i][0:5] , child = self.leftJointNames[i][6:11] , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i<9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:11]))

            if i == 9:
                pyrosim.Send_Joint(name = self.leftJointNames[i] , parent = str(self.leftJointNames[i][0:5]) , child = str(self.leftJointNames[i][6:12]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i=9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:12]))
            if i > 9:
                pyrosim.Send_Joint(name = self.leftJointNames[i] , parent = str(self.leftJointNames[i][0:6]) , child = str(self.leftJointNames[i][7:13]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i>9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:6]) + " | " + str(self.jointNames[i][7:13]))

        #Right Wing
        for i in range(self.numofLinks):
            
            pyrosim.Send_Cube(name=self.rightLinkNames[i], pos=self.rightLinkPositions[i] , size=linkSizeConstants[i], colorName = rightColorName[i], colorID = rightColorID[i])
            print("SENDING CUBE:")
            print("name: " + str(self.leftLinkNames[i]) + " size: "+ str(linkSizeConstants[i]))

        for i in range(self.numofLinks - 1):
            if i < 9:
                pyrosim.Send_Joint(name = self.rightJointNames[i] , parent = self.rightJointNames[i][0:5] , child = self.rightJointNames[i][6:11] , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i<9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:11]))

            if i == 9:
                pyrosim.Send_Joint(name = self.rightJointNames[i] , parent = str(self.rightJointNames[i][0:5]) , child = str(self.rightJointNames[i][6:12]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i=9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:5]) + " | " + str(self.jointNames[i][6:12]))
            if i > 9:
                pyrosim.Send_Joint(name = self.rightJointNames[i] , parent = str(self.rightJointNames[i][0:6]) , child = str(self.rightJointNames[i][7:13]) , type = "revolute", position = [0, 0.5, 0], jointAxis = jointAxisConstant)
                #print("Link i>9")
                #print(str(self.jointNames[i]) + " | " + str(self.jointNames[i][0:6]) + " | " + str(self.jointNames[i][7:13]))
        pyrosim.End()


    def Create_Brain(self):

        brainID = "brain" + str(self._myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)
    
        pyrosim.Start_NeuralNetwork(brainID)

        #Left Neurons
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = self.randomLink[0])
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = self.randomLink[1])
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = self.randomLink[2])
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = self.randomLink[3])

        leftRandomJoint1 = str(self.leftRandomLink[0]) + "_LeftLink" + str(int(self.randomLink[0][4]) + 1)
        leftRandomJoint2 = str(self.leftRandomLink[1]) + "_LeftLink" + str(int(self.leftRandomLink[1][4]) + 1)
        leftRandomJoint3 = str(self.leftRandomLink[2]) + "_LeftLink" + str(int(self.leftRandomLink[2][4]) + 1)
        leftRandomJoint4 = str(self.leftRandomLink[3]) + "_LeftLink" + str(int(self.leftRandomLink[3][4]) + 1)


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

        leftRandomJoint1 = str(self.rightRandomLink[0]) + "_LeftLink" + str(int(self.randomLink[0][4]) + 1)
        leftRandomJoint2 = str(self.randomLink[1]) + "_LeftLink" + str(int(self.randomLink[1][4]) + 1)
        leftRandomJoint3 = str(self.randomLink[2]) + "_LeftLink" + str(int(self.randomLink[2][4]) + 1)
        leftRandomJoint4 = str(self.randomLink[3]) + "_LeftLink" + str(int(self.randomLink[3][4]) + 1)


        print("LEFT RANDOM JOINTS:")
        print(leftRandomJoint1)
        print(leftRandomJoint2)
        print(leftRandomJoint3)
        print(leftRandomJoint4)

        pyrosim.Send_Motor_Neuron(name = 4 , jointName = leftRandomJoint1)
        pyrosim.Send_Motor_Neuron(name = 5 , jointName = leftRandomJoint2)
        pyrosim.Send_Motor_Neuron(name = 6 , jointName = leftRandomJoint3)
        pyrosim.Send_Motor_Neuron(name = 7 , jointName = leftRandomJoint4)

    
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



