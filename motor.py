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


        #self.amplitude = c.amplitude_Back
        #self.frequency = c.frequency_Back
        #self.offset = c.phaseOffset_Back

        #self.motorValues = self.amplitude * numpy.sin(self.frequency * numpy.linspace(0, 2 * numpy.pi, 10000) + self.offset)


    def Set_Value(self, robotID, desiredAngle):
        
        #pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = int(desiredAngle), maxForce = 50)
        #I Step 80: changed targetPosition for now. might need to revisit later.
        pyrosim.Set_Motor_For_Joint(bodyIndex = robotID, jointName = self.jointName, controlMode = p.POSITION_CONTROL, targetPosition = desiredAngle, maxForce = 50)


