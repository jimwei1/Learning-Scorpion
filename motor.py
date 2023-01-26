import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c
import robot as robot

class MOTOR:

    def __init__(self, jointName): #constructor

        self.jointName = jointName

        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude_Back
        self.frequency = c.frequency_Back
        self.offset = c.phaseOffset_Back
        
        self.motorValues = self.amplitude * numpy.sin(self.frequency * numpy.linspace(0, 2 * numpy.pi, 10000) + self.offset)

        if self.jointName == b'Torso_FrontLeg':
            self.frequency = self.frequency / 2


    def Set_Value(self, desiredAngle, robotID):
        pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = self.motorValues[desiredAngle], maxForce = 50)

    def Save_Values(self):
        numpy.save('/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/data/tempMotorValuesforH.npy', self.motorValues)

