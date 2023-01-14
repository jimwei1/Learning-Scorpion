import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

from world import WORLD
from robot import ROBOT
from sensor import SENSOR

class SIMULATION: #class name

    def __init__(self): #constructor

        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT()

        for i in range(c.timevalue):
            p.stepSimulation()
        
        p.disconnect()

        #simulation = SIMULATION()


    def Run(self):
        for i in range(c.timevalue):
                p.stepSimulation()
                self.SENSOR.backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
                self.SENSOR.frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

                targetAngles_Front = numpy.zeros(c.timevalue)
                targetAngles_Back = numpy.zeros(c.timevalue)

                targetAngles_Front[i] = c.amplitude_Front * numpy.sin(c.frequency_Front * i + c.phaseOffset_Front)
                targetAngles_Front[i] = c.amplitude_Back * numpy.sin(c.frequency_Back * i + c.phaseOffset_Back)

                pyrosim.Set_Motor_For_Joint(bodyIndex = self.robot.robotID, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = numpy.sin(targetAngles_Front[i]), maxForce = 50)
                pyrosim.Set_Motor_For_Joint(bodyIndex = self.robot.robotID, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = numpy.sin(targetAngles_Back[i]), maxForce = 50)

    def __del__(self):
        p.disconnect()