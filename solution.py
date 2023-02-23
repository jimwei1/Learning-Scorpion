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

        #self.Links = []
        #self.Joints = []

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.counter = 0
        self.Create_Brain()
        #self.Create_Ball()

        #os.system("python3 simulate.py " + directOrGUI + " " + str(self._myID) + " 2&>1 &")
        os.system("python3 simulate.py " + directOrGUI + " " + str(self._myID))

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

        self.linkSizeConstantsDict = [None] * self.numofLegs

        self.linkNamesDict = [None] * self.numofLegs

        self.jointNamesDict = [None] * self.numofLegs

        self.linkPositionsDict = [None] * self.numofLegs

        self.jointPositionsDict = [None] * self.numofLegs

        self.randomLinkDict = [None] * self.numofLegs

        self.colorIdDict = [None] * self.numofLegs

        self.colorNameDict = [None] * self.numofLegs

        for i in range(self.numofLegs):
            self.numofLinksDict[i] = random.randint(8, 12)

            print(self.numofLinksDict[i])

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
                self.jointNamesDict[i][x] = str(i) + "Link" + str(x) + "_" + str(i) + "Link" + str(x + 1)
            
            #Link Positions Dictionary, in the i location of array self.linkPositionsDict
            self.linkPositionsDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

            #Setting Link Positions based on Leg Number
            for x in self.linkPositionsDict[i]:
                
                #Neg z Leg
                if i == 0:
                    xPos = 0
                    yPos = 0
                    zPos = -0.5 #-1 *  self.linkSizeConstantsDict[i][x][2] // 2
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

                #Neg x Leg
                if i == 1:
                    xPos = -0.5 #-1 *  self.linkSizeConstantsDict[i][x][0] // 2  
                    yPos = 0
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

                #Pos z Leg
                if i == 2:
                    xPos = 0
                    yPos = 0
                    zPos = 0.5#self.linkSizeConstantsDict[i][x][2] // 2
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

                #Pos x Leg
                if i == 3:
                    xPos = 0.5 #self.linkSizeConstantsDict[i][x][0] // 2  
                    yPos = 0
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]
                
                #Neg y Leg
                if i == 4:
                    xPos = 0
                    yPos = -0.5 #-1 * self.linkSizeConstantsDict[i][x][0] // 2  
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]
                    
                #Pos y Leg
                if i == 5:
                    xPos = 0
                    yPos = 0.5 #self.linkSizeConstantsDict[i][x][0] // 2  
                    zPos = 0
                    self.linkPositionsDict[i][x] = [xPos, yPos, zPos]

            #Joint Positions: Same as Link Positions except -1 total size.
            numofJoints = int(self.numofLinksDict[i]) - 1
            self.jointPositionsDict[i] = dict.fromkeys(range(numofJoints), None)

            #Setting Link Positions based on Leg Number
            for x in self.jointPositionsDict[i]:
                
                #Neg z Leg
                if i == 0:
                    xPos = 0
                    yPos = 0
                    zPos = -0.5 #-1 *  self.linkSizeConstantsDict[i][x][2] // 2
                    self.jointPositionsDict[i][x] = [xPos, yPos, zPos]

                #Neg x Leg
                if i == 1:
                    xPos = -0.5 #-1 *  self.linkSizeConstantsDict[i][x][0] // 2  
                    yPos = 0
                    zPos = 0
                    self.jointPositionsDict[i][x] = [xPos, yPos, zPos]

                #Pos z Leg
                if i == 2:
                    xPos = 0
                    yPos = 0
                    zPos = 0.5#self.linkSizeConstantsDict[i][x][2] // 2
                    self.jointPositionsDict[i][x] = [xPos, yPos, zPos]

                #Pos x Leg
                if i == 3:
                    xPos = 0.5 #self.linkSizeConstantsDict[i][x][0] // 2  
                    yPos = 0
                    zPos = 0
                    self.jointPositionsDict[i][x] = [xPos, yPos, zPos]
                
                #Neg y Leg
                if i == 4:
                    xPos = 0
                    yPos = -0.5 #-1 * self.linkSizeConstantsDict[i][x][0] // 2  
                    zPos = 0
                    self.jointPositionsDict[i][x] = [xPos, yPos, zPos]
                    
                #Pos y Leg
                if i == 5:
                    xPos = 0
                    yPos = 0.5 #self.linkSizeConstantsDict[i][x][0] // 2  
                    zPos = 0
                    self.jointPositionsDict[i][x] = [xPos, yPos, zPos]


                
            #4 Random Links Generated, saved in location i of self.randomLinkDict
            self.randomLinkDict[i] = dict.fromkeys(range(4), None)

            #Make sure each Random Link is unique
            for x in self.randomLinkDict[i]:
                while True:
                    self.randomLinkDict[i][x] = str(i) + "Link" + str(random.randint(1, 11))
                    unique = True

                    for a in range(x):
                        if self.randomLinkDict[i][x] == self.randomLinkDict[i][a]:
                            unique = False

                    if unique == True:
                        break

            #Generate Color IDs, saved in self.colorIdDict[i], fill them in as default
            self.colorIdDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), '<color rgba="0 0 1.0 1.0"/>')
            
            for x in range(len(self.randomLinkDict[i])):
                linkNumArray = []

                for char in self.randomLinkDict[i][x]:
                    if char.isdigit():
                        linkNumArray.append(char)

                if len(linkNumArray) == 2:
                    linkNum = linkNumArray[1]
                
                if len(linkNumArray) == 3:
                    linkNum = linkNumArray[1-2]
                
                intLinkNum = int(linkNum)
                    
                self.colorIdDict[i][intLinkNum] = '<color rgba="0 1.0 0 1.0"/>'
                
                
            #Fill in ColorNames
            self.colorNameDict[i] = dict.fromkeys(range(self.numofLinksDict[i]), None)

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


        
        #Making Legs
        for leg in range(self.numofLegs):
    
            
            #Making Links for each Leg
            for link in range(self.numofLinksDict[leg]):
                pyrosim.Send_Cube(name=self.linkNamesDict[leg][link], pos = self.linkPositionsDict[leg][link], size = self.linkSizeConstantsDict[leg][link], colorName = self.colorNameDict[leg][link], colorID = self.colorIdDict[leg][link])

                print("SENDING CUBE:")
                print("Name: " + str(self.linkNamesDict[leg][link]) + " Size: " + str(self.linkSizeConstantsDict[leg][link]))
                print("Pos: "+ str(self.linkPositionsDict[leg][link]) + " ColorName: " + str(self.colorNameDict[leg][link]) + " ColorID: " + str(self.colorIdDict[leg][link]))

            #Making Joints for each Leg
            initJointPos = numpy.add(startPos, self.linkPositionsDict[leg][0])
            print("INITIAL JOINT POSITION:")
            print(initJointPos)
            firstChildName = str(leg) + "Link0"
            firstJointName = "Torso_" + firstChildName

            pyrosim.Send_Joint(name = firstJointName , parent= "Torso" , child = firstChildName , type = "revolute", position = initJointPos, jointAxis = jointAxisConstant)
            print("Creating Initial Joint:")
            print("Joint Name: " + str(firstJointName) + " Child: " + str(firstChildName))
            print("Joint Position: " +  str(initJointPos) + " Joint Axis: " + str(jointAxisConstant))
            for joint in range(self.numofLinksDict[leg] - 1):
                jointName = self.jointNamesDict[leg][joint]

                numArray = []

                #Get all digits from each Joint
                for char in jointName:
                    if char.isdigit():
                        numArray.append(char)
                
                #Joint looks like this: 1Link0_1Link1, or 1Link9_1link10, or 1Link10_1link11. Thus, if len(numArray) is 4, then we're getting [1] and [3]. If len(numArray) is 5, then we're getting [1][3,4]. If len(numArray is 5), then we're getting [1,2], [4,5].
                if len(numArray) == 4:
                    parentNum = numArray[1]
                    parentLink = str(leg) + "Link" + str(parentNum)
                    childNum = numArray[3]
                    childLink = str(leg) + "Link" + str(childNum)

                if len(numArray) == 5:
                    parentNum = numArray[1]
                    parentLink = str(leg) + "Link" + str(parentNum)
                    childArray = numArray[3:5]
                    childNum = int(''.join(map(str, childArray)))
                    childLink = str(leg) + "Link" + str(childNum)

                if len(numArray) == 6:
                    parentArray= numArray[1:3]
                    parentNum = int(''.join(map(str, parentArray)))
                    parentLink = str(leg) + "Link" + str(parentNum)
                    childArray = numArray[4:6]
                    childNum = int(''.join(map(str, childArray)))
                    childLink = str(leg) + "Link" + str(childNum)
                
                #Now, generate joints with those numbers.
                pyrosim.Send_Joint(name = self.jointNamesDict[leg][joint], parent = parentLink, child = childLink, type = "revolute", position = self.jointPositionsDict[leg][joint], jointAxis = jointAxisConstant)
                print("SENDING JOINT:")
                print("Joint Name: " + str(self.jointNamesDict[leg][joint]) + " Parent: " + str(parentLink) + " Child: " + str(childLink))
                print("Joint Position: " +  str(self.jointPositionsDict[leg][joint]) + " Joint Axis: " + str(jointAxisConstant))
                
        pyrosim.End()
        pass

    def Create_Brain(self):

        brainID = "brain" + str(self._myID) +".nndf"
        print("CREATING BRAIN:")
        print(brainID)
    
        pyrosim.Start_NeuralNetwork(brainID)

        randomJointNames = []
        self.allSensorsArray = []
        self.allJointsArray = []
        

        for leg in range(self.numofLegs):
            numofSensors = 0
            numofJoints = 0
            sensorsArray = []
            jointsArray = []
            #Creating Sensors with Random Links
            for randomLink in self.randomLinkDict[leg]:
                pyrosim.Send_Sensor_Neuron(name = self.counter, linkName = self.randomLinkDict[leg][randomLink])
                sensorsArray.append(self.counter)
                self.counter += 1
                numofSensors += 1
                print("CREATING SENSOR:")
                print("Sensor Name: " + str(self.counter) + " Link Name:" + str(self.randomLinkDict[leg][randomLink]))

            #Creating Random Joint Names:
            for randomLink in self.randomLinkDict[leg]:
                if len(str(self.randomLinkDict[leg][randomLink])) == 6:
                    Num = int(self.randomLinkDict[leg][randomLink][5])
                    randomJointNames.append(str(self.randomLinkDict[leg][randomLink]) + "_" + str(self.randomLinkDict[leg][randomLink][0:5]) + str(Num+ 1))
                
                if len(str(self.randomLinkDict[leg][randomLink])) == 7:
                    array = self.randomLinkDict[leg][randomLink][5:7]
                    Num = int(''.join(map(str, array)))
                    randomJointNames.append(str(self.randomLinkDict[leg][randomLink]) + "_" + str(self.randomLinkDict[leg][randomLink][0:5]) + str(Num + 1))
                
            for randomJoint in range(len(randomJointNames)):
                pyrosim.Send_Motor_Neuron(name = self.counter , jointName = randomJointNames[randomJoint])
                jointsArray.append(self.counter)
                self.counter += 1
                numofJoints += 1
                print("CREATING MOTOR:")
                print("Name: " + str(self.counter) + " JointName: " + str(randomJointNames[randomJoint]))
            
            self.allSensorsArray.append(numofSensors)
            self.allJointsArray.append(numofJoints)

            self.weights = numpy.random.rand(numofSensors, numofJoints)
            self.weights = (self.weights * 2) - 1

            #Row is # of sensors, Col is # of Joints
            for currentRow in range(numofSensors):
                for currentColumn in range(numofJoints):
                    pyrosim.Send_Synapse(sourceNeuronName = sensorsArray[currentRow] , targetNeuronName = jointsArray[currentColumn], weight = self.weights[currentRow][currentColumn])
                    print("SENDING SYNAPSE:")
                    print("SourceNeuronName: " + str(currentRow) + " TargetNeuronName: " + str(currentColumn + numofSensors) + " Weight: " + str(self.weights[currentRow][currentColumn]))

        pyrosim.End()

    def Mutate(self):
        for leg in range(self.numofLegs):
            numOfSensors = self.allSensorsArray[leg]
            numOfJoints = self.allJointsArray[leg]
            randomRow = random.randint(0, numOfSensors - 1)
            randomColumn = random.randint(0, numOfJoints - 1)

            self.weights[randomRow][randomColumn] = (random.random() * 2) - 1

    def Set_ID(self, nextAvailableID):
        self._myID = nextAvailableID



