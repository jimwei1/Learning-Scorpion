import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

from world import WORLD
from robot import ROBOT

class SIMULATION: #class name

    def __init__(self): #constructor

        #simulation = SIMULATION()
        self.world = WORLD()
        self.robot = ROBOT()

        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0,0,-9.8)
        pyrosim.Prepare_To_Simulate(self.robot.robotID)

        for i in range(c.timevalue):
            p.stepSimulation()
        
        p.disconnect()


    #def Run(self):
            #p.stepSimulation()
            #self.sensor.backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            #rontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

