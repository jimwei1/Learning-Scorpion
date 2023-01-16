import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c
import robot as robot

class MOTOR:

    def __init__(self): #constructor

        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude_Back
        self.frequency = c.frequency_Back
        self.offset = c.phaseOffset_Back
        
        self.motorValues = self.amplitude * numpy.sin(self.frequency + self.offset)


    def SetValue(self,t):
        pyrosim.Set_Motor_For_Joint(bodyIndex = (robot.self.robot), jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = self.motorValues[t], maxForce = 50)