import numpy as np
import pyrosim.pyrosim as pyrosim
#from robot import ROBOT
import random as random
import os as os
import time as time
import constants as c
#import pybullet as pybullet
from playsound import playsound

class SOLUTION:

    def __init__(self): #constructor
        self.Start_Simulation()

    def Start_Simulation(self):
        self.Create_Duck()

    def Create_Duck(self):
        pyrosim.Send_Cube(name="Torso", pos=[0, 2, 2] , size=[0.5,0.5,0.5])