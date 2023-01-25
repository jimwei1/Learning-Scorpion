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
        for t in range(c.timevalue):
                p.stepSimulation()
                SENSOR.Sense()
                self.robot.Think()
                self.robot.Act()



                targetAngles_Front = numpy.zeros(c.timevalue)
                targetAngles_Back = numpy.zeros(c.timevalue)

                targetAngles_Front[t] = c.amplitude_Front * numpy.sin(c.frequency_Front * t + c.phaseOffset_Front)
                targetAngles_Front[t] = c.amplitude_Back * numpy.sin(c.frequency_Back * t + c.phaseOffset_Back)

                
                
                #   p.gc.collect
    def __del__(self):
        p.disconnect()