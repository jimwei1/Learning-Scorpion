import numpy as np
import pyrosim.pyrosim as pyrosim
#from robot import ROBOT
import random as random
import os as os
import time as time
import constants as c
import pybullet as pybullet
from playsound import playsound

class SOLUTION:

    def __init__(self): #constructor
        self.myID = 0
        self.Start_Simulation()

    def Start_Simulation(self):
        physicsClient = p.connect(p.GUI)
        self.Create_World()
        self.Create_Duck()
        

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()
    def Create_Duck(self):
        pyrosim.Send_Cube(name="Torso", pos=[0, 2, 1.6] , size=[1,1,1], colorName = '<material name="Yellow">', colorID = '<color rgba="1.0 1.0 0 1.0"/>')
        #pyrosim.



quack = "/Users/jim/Documents/GitHub/CS-396-Artificial-Life-Bots/quack.wav"

playsound(quack)