import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as numpy
import random as random
import constants as c

class WORLD:

    def __init__(self): #constructor

        planeId = p.loadURDF("plane.urdf")

        p.loadSDF("world.sdf")

        robotID = p.loadURDF("body.urdf")