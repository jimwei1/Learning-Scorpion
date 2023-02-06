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

    def __init__(self, directOrGUI, solutionID): #constructor

        self.directOrGUI = directOrGUI

        if directOrGUI == "DIRECT":
            physicsClient = p.connect(p.DIRECT)
        elif directOrGUI == "GUI":
            physicsClient = p.connect(p.GUI)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0,0,-5.0)
        #p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT(solutionID)


    def Run(self):
        for t in range(c.timevalue):
                p.stepSimulation()
                self.robot.Sense(t)
                self.robot.Think()
                self.robot.Act(t)
                
                #if self.directOrGUI == "GUI":
                    #time.sleep(1/1000)
        
    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()